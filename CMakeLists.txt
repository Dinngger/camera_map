cmake_minimum_required(VERSION 2.8)
project( map_calibrate )

find_package( OpenCV REQUIRED )
include_directories( . ${OpenCV_INCLUDE_DIRS} )

aux_source_directory( tinyxml/ TinyXML_SRCS )

add_executable( map_calibrate map_calibrate.cc ${TinyXML_SRCS} )
target_link_libraries( map_calibrate ${OpenCV_LIBS} )

add_executable( cam_map_test cam_map_test.cc ${TinyXML_SRCS} )
target_link_libraries( cam_map_test ${OpenCV_LIBS} )