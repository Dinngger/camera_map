cmake_minimum_required(VERSION 2.8)
project( map_calibrate )

find_package( OpenCV REQUIRED )
include_directories( include ${OpenCV_INCLUDE_DIRS} /opt/MVS/include )
link_directories(/opt/MVS/lib/aarch64)

aux_source_directory( tinyxml/ TinyXML_SRCS )

# add_executable( map_calibrate map_calibrate.cc ${TinyXML_SRCS} )
# target_link_libraries( map_calibrate ${OpenCV_LIBS} )

# add_executable( cam_map_test cam_map_test.cc ${TinyXML_SRCS} )
# target_link_libraries( cam_map_test ${OpenCV_LIBS} )

add_executable( LinearEquationsTest linearEquations/LinearEquationsTest.cpp)

aux_source_directory( armorDetection/ ArmorDetection_SRCS )
add_executable( light_detection ${ArmorDetection_SRCS} )
target_link_libraries( light_detection ${OpenCV_LIBS} )

aux_source_directory( camera_ctl/ CameraCtl_SRCS )
add_executable( GrabOpenCV ${CameraCtl_SRCS} )
target_link_libraries( GrabOpenCV MvCameraControl ${OpenCV_LIBS} )
