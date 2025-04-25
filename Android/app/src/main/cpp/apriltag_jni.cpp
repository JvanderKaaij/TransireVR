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

static std::mutex apriltag_mutex;

// Static/global detector and family
static apriltag_detector_t* td = nullptr;
static apriltag_family_t* tf = nullptr;
static apriltag_detection_info_t info;
static int tagCount;
static std::vector<double> lastDetections;

extern "C" __attribute__((visibility("default")))
void start_camera_native() {
    camManager = Camera2Manager::create();
    auto configs = camManager->getCameraConfigs();

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Let's start");

    if (configs.empty()) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "No camera found!");
        return;
    }

    camera = camManager->openCamera(configs[0].id); // 0=Left, 1=Right
    if (!camera) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Failed to open camera");
        return;
    }

    for (const auto& cfg : configs) {
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Camera ID: %s (%dx%d) Position: %s",
                            cfg.id.c_str(), cfg.width, cfg.height,
                            cfg.position == Position::Left ? "Left" :
                            cfg.position == Position::Right ? "Right" : "Unknown");
    }

    Camera2Device::Callbacks cb;
    cb.onFrame = [](int64_t ts, std::vector<uint8_t> yuv) {
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Run Frame");

        const int width = 640;
        const int height = 480;

        std::vector<uint8_t> yuv_copy = std::move(yuv);  // Move into local var

        std::thread([yuv = std::move(yuv_copy), width, height]() mutable {
            std::lock_guard<std::mutex> lock(apriltag_mutex);
            detect_apriltags(yuv.data(), width, height);
        }).detach();
    };

    cb.onError = [](const std::string& msg) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Camera Error: %s", msg.c_str());
    };

    if (!camera->start(cb)) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Failed to start camera");
    }
}

extern "C" __attribute__((visibility("default")))
void init_detector(float tagsize, int tagFamily, float focalLengthX, float focalLengthY, float focalCenterX, float focalCenterY){
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
    info.fx = focalLengthX;
    info.fy = focalLengthY;
    info.cx = focalCenterX;
    info.cy = focalCenterY;

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Initializing: tagsize=%.3f fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
//                        info.tagsize, info.fx, info.fy, info.cx, info.cy);

    // Optional: tune detection parameters here
    td->quad_decimate = 2.0;
    td->nthreads = 4;
    td->refine_edges = true;
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

//    int write_result = image_u8_write_pnm(&img, "/sdcard/Android/data/com.samples.passthroughcamera/files/frame.pnm");
//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Saved frame.pnm: %s", write_result == 0 ? "success" : "FAILED");

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Running detection on %dx%d image", width, height);

    auto t0 = std::chrono::high_resolution_clock::now();
    zarray_t* detections = apriltag_detector_detect(td, &img);
    auto t1 = std::chrono::high_resolution_clock::now();
    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "TIMING apriltag_detector_detect took: %lldms",
                        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());

    int count = zarray_size(detections);

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Detected %d tags", count);

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Pixel[0]=%d, center=%d, last=%d",
//                        image_data[0],
//                        image_data[(height/2) * width + (width/2)],
//                        image_data[height * width - 1]
//    );

    std::vector<TagPose> tagResults;

    // Optional: free detections here (don't leak memory!)
    for (int i = 0; i < count; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        info.det = det;
        apriltag_pose_t pose;

        auto t0 = std::chrono::high_resolution_clock::now();
        estimate_tag_pose(&info, &pose);
        auto t1 = std::chrono::high_resolution_clock::now();
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "TIMING estimate_tag_pose took: %lldms",
                            std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());

        TagPose result;
        result.id = det->id;
        for (int j = 0; j < 3; j++) {
            result.translation[j] = pose.t->data[j];
        }
        __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Tag Z Position: %f", result.translation[2]);

        for (int j = 0; j < 9; j++) {
            result.rotation[j] = pose.R->data[j];
        }

        tagResults.push_back(result);

        //Destroy after use
        matd_destroy(pose.R);
        matd_destroy(pose.t);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);

    lastDetections.clear();
    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Tag Results: %d tags", tagResults.size());

    tagCount = count;

    for (const auto& pose : tagResults) {
        lastDetections.push_back(static_cast<double>(pose.id));       // ID as double (safe)
        lastDetections.insert(lastDetections.end(), pose.translation, pose.translation + 3);
        lastDetections.insert(lastDetections.end(), pose.rotation, pose.rotation + 9);
    }

    return static_cast<int>(lastDetections.size());
}

extern "C" __attribute__((visibility("default")))
const double* get_latest_poses() {
    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Retrieving Last Detections amount: %d", lastDetections.size());
    return lastDetections.data();
}


bool save_grayscale_pgm(const std::string& filename, const uint8_t* buffer, int width, int height) {
    FILE* f = fopen(filename.c_str(), "wb");
    if (!f) return false;

    fprintf(f, "P5\n%d %d\n255\n", width, height);  // PGM header
    fwrite(buffer, 1, width * height, f);
    fclose(f);

    return true;
}


extern "C" __attribute__((visibility("default")))
int count_apriltags(){
    return tagCount;
}

extern "C" __attribute__((visibility("default")))
void stop_camera_native() {
    if (camera) {
        camera->stop();
        camera = nullptr;
    }
    if (camManager) {
        camManager->shutdown();
        camManager = nullptr;
    }
}

