cmake_minimum_required(VERSION 2.8)
project(pose_solver)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 3 not found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(Pangolin REQUIRED)
if(NOT Pangolin_FOUND)
   message(FATAL_ERROR "Pangolin not found.")
endif()

find_package(Ceres REQUIRED)
if(NOT Ceres_FOUND)
   message(FATAL_ERROR "Ceres not found.")
endif()

set(Torch_DIR /opt/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
if(NOT Torch_FOUND)
   message(FATAL_ERROR "Torch not found.")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(FATAL_ERROR "No suitable OpenMP found.")
endif()

include_directories(
${TORCH_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
)

link_directories(
/usr/local/lib
${TORCH_LIBRARY_DIRS}
)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

aux_source_directory(src/ PROJ_SRC)

add_executable(viewer
pose_estimate.cc
${PROJ_SRC}
)

target_link_libraries(viewer
${Pangolin_LIBRARIES}
${OpenCV_LIBS}
${CERES_LIBRARIES}
${TORCH_LIBRARIES}
)

add_executable(dataset_generator
dataset_generator.cc
${PROJ_SRC}
)

target_link_libraries(dataset_generator
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${CERES_LIBRARIES}
${TORCH_LIBRARIES}
)
