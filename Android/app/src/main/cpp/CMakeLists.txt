cmake_minimum_required(VERSION 3.4.1)
project(apriltagnative)

file(GLOB APRILTAG_COMMON_SRC
        "apriltag/common/*.c"
)

set(APRILTAG_SRC
        apriltag/apriltag.c
        apriltag/tag36h11.c
        apriltag/tagStandard41h12.c
        apriltag/tag16h5.c
        apriltag/apriltag_quad_thresh.c
        apriltag/apriltag_pose.c
        ${APRILTAG_COMMON_SRC}
)

add_library(apriltagnative SHARED
        apriltag_jni.cpp
        camera2impl.cpp
        ndk/helpers.cpp
        ${APRILTAG_SRC}
)

include_directories(
        apriltag
        apriltag/common
)

find_library(log-lib log)
find_library(camera2ndk-lib camera2ndk)
find_library(mediandk-lib mediandk)
find_library(android-lib android)

target_link_libraries(apriltagnative
        ${log-lib}
        ${camera2ndk-lib}
        ${mediandk-lib}
        ${android-lib}
)
