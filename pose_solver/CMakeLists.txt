cmake_minimum_required(VERSION 2.8)
project(pose_solver)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 or C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
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
# message(STATUS ${EIGEN3_INCLUDE_DIRS})

#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#FIND_PACKAGE(G2O REQUIRED)
#find_package(CSparse REQUIRED)
#find_package(Cholmod REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
#${G2O_INCLUDE_DIR}
#${CSPARSE_INCLUDE_DIR}
#${CHOLMOD_INCLUDE_DIR}
)

#message(STATUS "Pangolin include path: " ${Pangolin_INCLUDE_DIR})

link_directories(
/usr/local/lib
#/mine/g2o/lib
)
#SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

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
#${G2O_LIBS}
#${CHOLMOD_LIBRARIES}

#this is only for hqy's computer with stupid anaconda.
#/usr/lib/x86_64-linux-gnu/libtiff.so.5.2.4
)
