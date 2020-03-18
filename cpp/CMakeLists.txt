cmake_minimum_required(VERSION 2.8.4)
project(traj_gen CXX )
add_compile_options(-std=c++11)

include(GNUInstallDirs) # holds CMAKE_INSTALL_*DIR information
# This should exist to define library "traj_gen". The information will be used with set_target_properties
add_library(traj_gen SHARED src/TrajGen.cpp)


# We need to add dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(qpOASES REQUIRED)

set_target_properties(traj_gen PROPERTIES
        VERSION 2.0.0 SOVERSION 1 PUBLIC_HEADER include/traj_gen2/TrajGen.hpp
        )

target_link_libraries(traj_gen qpOASES)

# We do not want /eigen3/ when including Eigen/Core in our header files.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# The role of the below is to packaging all the include (including myself) necessary for our project
target_include_directories(traj_gen PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR})


# Let us install the lib and include along with .cmake file to be discovered in other packages.
install(TARGETS traj_gen EXPORT TrajGenConfig
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/traj_gen/)

configure_file(cmake/trajgen.pc.in trajgen.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/trajgen.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

install(EXPORT TrajGenConfig DESTINATION share/TrajGen/cmake)
export(TARGETS ${PROJECT_NAME} FILE TrajGenConfig.cmake)
