cmake_minimum_required(VERSION 3.4.1)
project(apriltagnative)

file(GLOB APRILTAG_COMMON_SRC
        "apriltag/common/*.c"
)

set(APRILTAG_SRC
        apriltag/apriltag.c
        apriltag/tag36h11.c
        apriltag/tagStandard41h12.c
        apriltag/apriltag_quad_thresh.c
        apriltag/apriltag_pose.c
        ${APRILTAG_COMMON_SRC}
)

add_library(apriltagnative SHARED
        apriltag_jni.cpp   # JNI wrapper
        ${APRILTAG_SRC}
)

include_directories(
        apriltag
        apriltag/common
)

find_library(log-lib log)

target_link_libraries(apriltagnative
        ${log-lib}
)