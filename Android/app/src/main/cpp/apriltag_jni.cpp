#include <cstdio>
#include <cstdlib>
#include <vector>
#include <android/log.h>
#include "image_u8.h"
#include "apriltag.h"
#include "tagStandard41h12.h"
#include "apriltag_pose.h"

extern "C" __attribute__((visibility("default")))
const char* stringFromJNI() {
    return "Unity-native integration FTW!";
}

struct TagPose {
    int id;
    double translation[3];  // x, y, z
    double rotation[9];     // 3x3 matrix (row-major)
};

// Static/global detector and family
static apriltag_detector_t* td = nullptr;
static apriltag_family_t* tf = nullptr;
static apriltag_detection_info_t info;
static std::vector<double> lastDetections;


extern "C" __attribute__((visibility("default")))
void init_detector(float tagsize, float focalLengthX, float focalLengthY, float focalCenterX, float focalCenterY){
    if (td != nullptr) return;  // already initialized
    td = apriltag_detector_create();
    tf = tagStandard41h12_create();
    apriltag_detector_add_family(td, tf);

    info.tagsize = tagsize; //The size of the tag in meters
    info.fx = focalLengthX;
    info.fy = focalLengthY;
    info.cx = focalCenterX;
    info.cy = focalCenterY;

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Initializing: tagsize=%.3f fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
//                        info.tagsize, info.fx, info.fy, info.cx, info.cy);

    // Optional: tune detection parameters here
    td->quad_decimate = 1.0;
    td->nthreads = 8; //Oculus has 8 cores
    //td->quad_sigma = 0.0;
    //td->refine_edges = 1;
}

extern "C" __attribute__((visibility("default")))
int detect_apriltags(const uint8_t* rgba, int width, int height) {
    if (td == nullptr) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Detector not initialized! Call init_detector()");
        return -1;
    }

    if (!rgba) {
        __android_log_print(ANDROID_LOG_ERROR, "AprilTagNative", "Null image buffer!");
        return -1;
    }

    std::vector<uint8_t> grayscale(width * height);

    for (int y = 0; y < height; ++y) {
        int flipped_y = height - 1 - y;
        for (int x = 0; x < width; ++x) {
            int src_idx = (y * width + x) * 4;
            int dst_idx = flipped_y * width + x;

            int r = rgba[src_idx];
            int g = rgba[src_idx + 1];
            int b = rgba[src_idx + 2];

            grayscale[dst_idx] = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
        }
    }

    image_u8_t img = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = const_cast<uint8_t*>(grayscale.data())
    };

//    int write_result = image_u8_write_pnm(&img, "/sdcard/Android/data/com.samples.passthroughcamera/files/frame.pnm");
//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Saved frame.pnm: %s", write_result == 0 ? "success" : "FAILED");

//    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Running detection on %dx%d image", width, height);

    zarray_t* detections = apriltag_detector_detect(td, &img);
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
    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Tag Results: %d tags", tagResults.size());

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