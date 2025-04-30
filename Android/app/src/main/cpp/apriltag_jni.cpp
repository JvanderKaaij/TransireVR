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
void start_camera_native(float tagsize, int tagFamily) {
    camManager = Camera2Manager::create();

    int targetWidth = 640;
    int targetHeight = 480;

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

    camera = camManager->openCamera(leftCamConfig.id, targetWidth, targetHeight); // 0=Left, 1=Right

    if (!camera) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Failed to open camera");
        return;
    }

    init_detector(tagsize, tagFamily, leftCamConfig);

    Camera2Device::Callbacks cb;
    cb.onFrame = [width, height](int64_t ts, std::vector<uint8_t> yuv) {
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

    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "About to start Init Detector!");

}

void init_detector(float tagsize, int tagFamily, Camera2Configuration config){
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

    zarray_t* detections = apriltag_detector_detect(td, &img);

    int count = zarray_size(detections);

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

        //Destroy after use
        matd_destroy(pose.R);
        matd_destroy(pose.t);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);

    lastDetections.clear();
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
    return lastDetections.data();
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

