cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_find_ros_package(actionlib)
#include(${actionlib_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



rosbuild_add_executable(transformer src/transformer.cpp)
rosbuild_add_executable(tableFilter src/tableFilter.cpp)
rosbuild_add_executable(driver src/driver.cpp)
rosbuild_add_executable(bookFinder src/bookFinder.cpp src/util.cpp)
#include_directories(bookFinder `pkg-config opencv --cflags`)
#include_directories(/usr/local/include/opencv)
#target_link_libraries(bookFinder cv)


#rosbuild_add_executable(testing src/testroutine.cpp)
#rosbuild_add_executable(testFunction src/testFunction.cpp src/lib/angle_diff.cpp src/lib/angle_mean.cpp src/lib/book_edge_dist.cpp src/lib/edge2polar.cpp src/lib/line_clip.cpp src/lib/line_point_dist.cpp)
