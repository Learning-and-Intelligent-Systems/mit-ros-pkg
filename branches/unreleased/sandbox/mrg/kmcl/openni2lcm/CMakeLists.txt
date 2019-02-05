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


link_libraries(bot2-core lcmtypes_bot2-core lcmtypes_openni-lcm  bot-core glib-2.0 lcm isam z)

include_directories(/home/mfallon/marine/projects/kmcl/build/include)
link_directories(/home/mfallon/marine/projects/kmcl/build/lib)
#include_directories(/home/hordurj/sandbox/vo/build/include /home/hordurj/sandbox/vo/build/rgbd-odometry /home/hordurj/sandbox/vo/build/rgbd-odometry/extern/include)
#link_directories(/home/hordurj/sandbox/vo/build/lib)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(openni2lcm openni2lcm.cpp)
#target_link_libraries(example ${PROJECT_NAME})
