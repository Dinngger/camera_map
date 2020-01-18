CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( g2o_test )

find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CHOLMOD_INCLUDE_DIRS})

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
FIND_PACKAGE(G2O)
find_package(CSparse REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIRS})
message(STATUS ${CSPARSE_INCLUDE_DIRS})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

ADD_EXECUTABLE(g2o_test ba_demo.cpp)
target_link_libraries(g2o_test ${G2O_LIBS})
