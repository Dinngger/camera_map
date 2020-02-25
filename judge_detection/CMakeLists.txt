cmake_minimum_required(VERSION 2.8)
project( judge_detection )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} include ../pose_estimation/include )

add_executable( judge_detection
./judge_detection.cc
)

target_link_libraries( judge_detection ${OpenCV_LIBS} )
