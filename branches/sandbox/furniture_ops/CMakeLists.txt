cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#for openMP:
set(CMAKE_CXX_FLAGS  "-fopenmp")


#common commands for building c++ executables and libraries
rosbuild_add_library(pclhelper src/pcl_helpers.cpp)


#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable (bag_convert src/bag_read.cpp)
#target_link_libraries (bag_convert pcl_io)
#rosbuild_add_executable (cloud_grabber src/cloud_grabber.cpp)
#target_link_libraries (cloud_grabber pcl_io)
rosbuild_add_boost_directories()
#rosbuild_link_boost(cloud_grabber thread)
rosbuild_add_executable (pcd_publisher src/publishpcd.cpp)

#rosbuild_add_executable (pcd_npublisher src/publishpcd_wnormals.cpp)
#target_link_libraries (pcd_npublisher pclhelper)

#rosbuild_add_executable (segment_furniture src/segment_furniture.cpp)
#rosbuild_link_boost(segment_furniture thread)
#target_link_libraries (segment_furniture pclhelper)

#rosbuild_add_executable (voxel_filter src/filtervoxel.cpp)
#target_link_libraries (voxel_filter pcl_io pcl_filters)

#rosbuild_add_executable (pcl_test src/pcl_test.cpp)
#target_link_libraries (pcl_test pclhelper)

#rosbuild_add_executable (pcd_to_ply src/pcd_to_ply.cpp)
#target_link_libraries (pcd_to_ply pclhelper)

#rosbuild_add_executable (convertpcd src/convertpcd.cpp)
##rosbuild_add_executable (occupancy src/occupancy.cpp)


#rosbuild_add_executable (testradius src/test_radiusSearch.cpp)


#rosbuild_add_executable (pcl_test3 src/pcl_test3.cpp)
#rosbuild_add_executable (pcl_test2 src/pcl_test2.cpp)
#rosbuild_add_executable (circle_planner src/circle_planner.cpp)
#rosbuild_add_executable (align_clouds src/align_clouds.cpp)
#rosbuild_add_executable (cluster_furniture src/cluster_furniture.cpp)