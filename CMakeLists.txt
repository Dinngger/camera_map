cmake_minimum_required(VERSION 2.8)
project( map_calibrate )

find_package( OpenCV REQUIRED )

# for arm
#include_directories( include ${OpenCV_INCLUDE_DIRS} /opt/MVS/include )
#link_directories(/opt/MVS/lib/aarch64)

# for x86_64
include_directories( include ${OpenCV_INCLUDE_DIRS} /opt/MVS/include )
link_directories(/opt/MVS/lib/64)

aux_source_directory( tinyxml/ TinyXML_SRCS )

# add_executable( map_calibrate map_calibrate.cc ${TinyXML_SRCS} )
# target_link_libraries( map_calibrate ${OpenCV_LIBS} )

# add_executable( cam_map_test cam_map_test.cc ${TinyXML_SRCS} )
# target_link_libraries( cam_map_test ${OpenCV_LIBS} )

#add_executable( LinearEquationsTest linearEquations/LinearEquationsTest.cpp)

add_executable( GrabOpenCV
camera_ctl/GrabOpenCV.cc
camera_ctl/CameraCtl.cc
)
target_link_libraries( GrabOpenCV MvCameraControl ${OpenCV_LIBS} )
