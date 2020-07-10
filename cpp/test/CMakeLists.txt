cmake_minimum_required(VERSION 2.8.3)
project(test_pin )
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)
# append the Find*.cmake files (qpOASES)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake/")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_BINARY_DIR}/../../include)

find_package(TrajGen REQUIRED)
add_executable(test_pin test.cpp)
target_link_libraries( test_pin ${EIGEN3_LIBS}  traj_gen )

add_executable(simple_test_pin simple_test.cpp)
target_link_libraries( simple_test_pin ${EIGEN3_LIBS}  traj_gen )