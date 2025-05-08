#include <cstdio>
#include <cstdlib>
#include <vector>
#include <android/log.h>
#include <__thread/thread.h>
#include "image_u8.h"
#include "apriltag.h"
#include "tagStandard41h12.h"
#include "tag16h5.h"
#include "apriltag_pose.h"
#include "camera2.h"
#include "apriltag_jni.h"
#include <mutex>
#include <time.h> // For clock_gettime
#include <__thread/this_thread.h>
#include "threadpool.h"

static std::mutex apriltag_mutex;

// Static/global detector and family
static apriltag_detector_t* td = nullptr;
static apriltag_family_t* tf = nullptr;
static apriltag_detection_info_t info;
static int tagCount;
static std::vector<double> lastDetections;
static ThreadPool pool(4);


extern "C" __attribute__((visibility("default")))
void start_camera_native(float tagsize, int tagFamily, int targetWidth, int targetHeight) {
    camManager = Camera2Manager::create();

    auto configs = camManager->getCameraConfigs(targetWidth, targetHeight);

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Let's start");

    if (configs.empty()) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "No camera found!");
        return;
    }

    for (const auto& cfg : configs) {
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Camera ID: %s (%dx%d) Position: %s",
                            cfg.id.c_str(), cfg.width, cfg.height,
                            cfg.position == Position::Left ? "Left" :
                            cfg.position == Position::Right ? "Right" : "Unknown");
    }

    Camera2Configuration leftCamConfig = configs[0];

    const int width = leftCamConfig.width;
    const int height = leftCamConfig.height;

    for (int i = 0; i < 5; ++i) {
        camera = camManager->openCamera(leftCamConfig.id, targetWidth, targetHeight);
        if (camera) break;
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Retrying camera open (attempt %d)", i + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!camera) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Failed to open camera after 5 attempts");
        return;
    }

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "About to start Init Detector!");
    init_detector(tagsize, tagFamily, leftCamConfig);

    Camera2Device::Callbacks cb;
//    cb.onFrame = [width, height](int64_t ts, std::vector<uint8_t> yuv) {
//
//        std::vector<uint8_t> yuv_copy = std::move(yuv);  // Move into local var
//
//        std::thread([yuv = std::move(yuv_copy), width, height]() mutable {
//            std::lock_guard<std::mutex> lock(apriltag_mutex);
//            detect_apriltags(yuv.data(), width, height);
//        }).detach();
//    };

    // Create the pool (e.g., 4 threads)
    // Use it in your callback
    cb.onFrame = [width, height](int64_t ts, std::vector<uint8_t> yuv) {
        pool.enqueue([yuv = std::move(yuv), width, height]() mutable {
            std::lock_guard<std::mutex> lock(apriltag_mutex);
            detect_apriltags(yuv.data(), width, height);
        });
    };

    cb.onError = [](const std::string& msg) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Camera Error: %s", msg.c_str());
    };

    try {
        if (!camera->start(cb)) {
            __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Failed to start camera (unknown reason)");
        }
    } catch (const std::exception& e) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Exception while starting camera: %s", e.what());
    } catch (...) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Unknown exception while starting camera");
    }


}

void init_detector(float tagsize, int tagFamily, Camera2Configuration config){
    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Initializing detector!");

    if (td != nullptr) return;  // already initialized
    td = apriltag_detector_create();

    if(tagFamily == 0){
        tf = tag16h5_create();
    }else if(tagFamily == 5){
        tf = tagStandard41h12_create();
    }else{

    }
    apriltag_detector_add_family(td, tf);

    info.tagsize = tagsize; //The size of the tag in meters
    info.fx = config.fx;
    info.fy = config.fy;
    info.cx = config.cx;
    info.cy = config.cy;

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Initializing: tagsize=%.3f fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
//                        info.tagsize, info.fx, info.fy, info.cx, info.cy);

    // Optional: tune detection parameters here
    td->quad_decimate = 2.0;
    td->nthreads = 4;
    td->refine_edges = true;

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Finish Initializing detector!");
}

int detect_apriltags(const uint8_t* grayscale_data, int width, int height) {
    if (td == nullptr) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Detector not initialized! Call init_detector()");
        return -1;
    }

    if (!grayscale_data) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Null image buffer!");
        return -1;
    }

    image_u8_t img = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = const_cast<uint8_t*>(grayscale_data)
    };

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Running detection on %dx%d image", width, height);

    // Start timing
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    zarray_t* detections = apriltag_detector_detect(td, &img);

    // End timing
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    long elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000 +
                      (end_time.tv_nsec - start_time.tv_nsec) / 1000000;

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Detection took %ld ms", elapsed_ms);

    int count = zarray_size(detections);

    std::vector<TagPose> tagResults;

    for (int i = 0; i < count; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        info.det = det;
        apriltag_pose_t pose;

        estimate_tag_pose(&info, &pose);

        TagPose result;
        result.id = det->id;
        for (int j = 0; j < 3; j++) {
            result.translation[j] = pose.t->data[j];
        }
        for (int j = 0; j < 9; j++) {
            result.rotation[j] = pose.R->data[j];
        }

        tagResults.push_back(result);

        matd_destroy(pose.R);
        matd_destroy(pose.t);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);

    lastDetections.clear();
    tagCount = count;

    for (const auto& pose : tagResults) {
        lastDetections.push_back(static_cast<double>(pose.id));
        lastDetections.insert(lastDetections.end(), pose.translation, pose.translation + 3);
        lastDetections.insert(lastDetections.end(), pose.rotation, pose.rotation + 9);
    }

    return static_cast<int>(lastDetections.size());
}

extern "C" __attribute__((visibility("default")))
const double* get_latest_poses() {
    return lastDetections.data();
}

extern "C" __attribute__((visibility("default")))
int count_apriltags(){
    return tagCount;
}

extern "C" __attribute__((visibility("default")))
void stop_camera_native() {

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "STOP Native Camera");


    // Stop and reset the camera
    if (camera) {
        camera->stop();
        camera = nullptr;
    }

    // Shutdown the camera manager
    if (camManager) {
        camManager->shutdown();
        camManager = nullptr;
    }

    // Free the detector and tag family
    if (td) {
        apriltag_detector_destroy(td);
        td = nullptr;
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Cleaned AprilTag Detector");
    }

    if (tf) {
        if (info.tagsize == 0) {
            tag16h5_destroy(tf);
        } else if (info.tagsize == 5) {
            tagStandard41h12_destroy(tf);
        }
        tf = nullptr;
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Cleaned Tag Family");
    }

    // Clear the last detections and tag count
    lastDetections.clear();
    tagCount = 0;

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Camera and detector resources successfully cleaned up.");
}