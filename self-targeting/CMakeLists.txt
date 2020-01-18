# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(Task1)
set(CMAKE_BUILD_TYPE "Release")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)  
include_directories(. ${OpenCV_INCLUDE_DIRS} /opt/MVS/include
    ./include
    ./include/universal
    ./include/detection
)
link_directories(
    /opt/MVS/lib/64
    /opt/MVS/lib/32
)
# Declare the executable target built from your sources
add_executable(Task1 testAngle.cpp)
TARGET_LINK_LIBRARIES(Task1
    MvCameraControl
    ${OpenCV_LIBS}
    libMvCameraControl.so
)
