cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(draw_square src/draw_square.cpp)
rosbuild_add_executable(pr2_write_pencil src/pr2_write_pencil.cpp)
rosbuild_add_executable(pr2_write_mit src/pr2_write_mit.cpp)
rosbuild_add_executable(pr2_turn_page src/pr2_turn_page.cpp)
rosbuild_add_executable(pr2_cut_cake src/pr2_cut_cake.cpp)
rosbuild_add_executable(pr2_wipe_screen src/pr2_wipe_screen.cpp)
rosbuild_add_executable(stop_trajectory src/stop_trajectory.cpp)
rosbuild_add_executable(pr2_sweep src/pr2_sweep.cpp)
rosbuild_add_executable(pr2_stiff_point src/pr2_stiff_point.cpp)
