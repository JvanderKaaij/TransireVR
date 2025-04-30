#ifndef APRILTAG_JNI_H
#define APRILTAG_JNI_H

#include <vector>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Struct representing the pose of a detected AprilTag
struct TagPose {
    int id;
    double translation[3];  // x, y, z
    double rotation[9];     // 3x3 matrix (row-major)
};


static std::unique_ptr<Camera2Manager> camManager;
static std::shared_ptr<Camera2Device> camera;

// Starts the camera and AprilTag detection pipeline
// tagsize: physical tag size in meters
// tagFamily: 0 = tag16h5, 5 = tagStandard41h12

void start_camera_native(float tagsize, int tagFamily);

// Initializes the AprilTag detector with camera intrinsics
// focalLengthX/Y: focal lengths in pixels
// focalCenterX/Y: principal point coordinates in pixels
void init_detector(float tagsize, int tagFamily, Camera2Configuration config);

// Processes a grayscale image and runs AprilTag detection
// grayscale_data: pointer to Y channel (grayscale image buffer)
// width/height: dimensions of the image
// Returns the number of elements in the latest detection result (id + pose data)
int detect_apriltags(const uint8_t* grayscale_data, int width, int height);

// Returns a pointer to the most recent tag poses
// Data layout: [id, tx, ty, tz, r00..r22] per tag, flat array
const double* get_latest_poses();

// Returns how many tags were detected in the most recent frame
int count_apriltags();

// Stops the camera and shuts down the AprilTag pipeline
void stop_camera_native();

#ifdef __cplusplus
}
#endif

#endif // APRILTAG_JNI_H
