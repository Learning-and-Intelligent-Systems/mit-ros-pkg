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

add_definitions(-Wall -O3 -mfpmath=sse -mmmx -msse -msse2 -msse3)# -fopenmp)
rosbuild_add_library(${PROJECT_NAME} src/viewer.cpp src/util.cpp src/visualization.cpp)

include_directories(/home/hordurj/marine/projects/lcmtypes/src/lcmtypes)
#link_directories(/home/hordurj/marine/projects/lcmtypes/lib)
include_directories(/home/hordurj/sandbox/vo/build/include /home/hordurj/sandbox/vo/build/rgbd-odometry /home/hordurj/sandbox/vo/build/rgbd-odometry/extern/include)
#link_directories(/home/hordurj/sandbox/vo/build/lib)
#link_libraries(bot2-core lcmtypes_bot2-core bot-core lcm rgbd-odometry mrtypes isam)
#rosbuild_add_executable(neuroslam src/neuroslam.cpp src/util.cpp src/visualization.cpp src/viewer.cpp)

