SET(CMAKE_C_COMPILER gcc.exe )
SET(CMAKE_CXX_COMPILER g++.exe )

cmake_minimum_required(VERSION 2.8)
project( judge_detection )

set( OpenCV_DIR C:/opencv/build ) 

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} include )

LINK_DIRECTORIES( C:/opencv/build/install/x64/mingw/lib )

add_executable( judge_detection
./judge_detection.cc
)

target_link_libraries( judge_detection ${OpenCV_LIBS} )
