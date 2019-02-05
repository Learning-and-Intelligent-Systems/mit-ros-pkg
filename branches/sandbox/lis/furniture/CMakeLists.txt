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

#rosbuild_add_executable(tracker src/tracker.cpp)
#rosbuild_add_executable(table_detector src/table_detector.cpp)

rosbuild_add_executable(table_detector_standalone src/table_detector_standalone.cpp)
#rosbuild_add_executable(table_detector_with_normals src/table_detector_with_normals.cpp src/util.cpp)

#rosbuild_add_executable(table_tracker src/table_tracker.cpp)

#rosbuild_add_executable(table_detector_beta src/table_detector_beta.cpp src/util.cpp)
#rosbuild_add_executable(steady_publisher src/steady_publisher.cpp)

#rosbuild_add_executable(pcl_subscriber src/pcl_subscriber.cpp)
rosbuild_add_executable(table_tracker src/table_tracker.cpp)
rosbuild_add_executable(multiplanar_tracker src/multiplanar_tracker.cpp)
#rosbuild_add_executable(chair_detector_standalone src/chair_detector_standalone.cpp)
#rosbuild_add_executable(chair_detector2 src/chair_detector_standalone.cpp)

#rosbuild_add_executable(table_analyzer src/table_analyzer.cpp src/util.cpp)
#rosbuild_add_executable(color_tracker src/color_tracker.cpp src/util.cpp)
#rosbuild_add_executable(scene_segmentation src/scene_segmentation.cpp)
#rosbuild_add_compile_flags(scene_segmentation "-pg")
#rosbuild_add_link_flags(scene_segmentation "-pg")
#rosbuild_add_executable(automatic_model_acquisition src/automatic_model_acquisition.cpp)
