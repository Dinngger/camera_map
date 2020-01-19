cmake_minimum_required(VERSION 2.8)
project(SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# message(STATUS ${EIGEN3_INCLUDE_DIRS})

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../include
${PROJECT_SOURCE_DIR}/../self_targeting/include/detection
${PROJECT_SOURCE_DIR}/../self_targeting/include/universal
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
/opt/MVS/include
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

aux_source_directory(src/ VIEWER_SRC)
link_directories(/opt/MVS/lib/64)

add_executable(viewer
pose_estimate.cc
${PROJECT_SOURCE_DIR}/../armorDetection/ArmorDetect.cpp
${VIEWER_SRC}
)

target_link_libraries(viewer
MvCameraControl
${Pangolin_LIBRARIES}
${OpenCV_LIBS}
)

add_executable(pnp_solver
pnp_solver.cc
${PROJECT_SOURCE_DIR}/../armorDetection/ArmorDetect.cpp
)

target_link_libraries(pnp_solver
MvCameraControl
${OpenCV_LIBS}
)