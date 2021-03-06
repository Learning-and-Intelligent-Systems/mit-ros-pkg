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
rosbuild_add_boost_directories ()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories(${wxWidgets_INCLUDE_DIRS})
#find_package (VTK REQUIRED)
#include(${VTK_USE_FILE})


rosbuild_add_executable(ply_to_pcd src/ply_to_pcd.cpp src/plyfile.c)
target_link_libraries(ply_to_pcd pcl_io)

rosbuild_add_executable(pcd_to_ply src/pcd_to_ply.cpp)
target_link_libraries(pcd_to_ply pcl_io)

rosbuild_add_library(${PROJECT_NAME} src/detection.cpp src/optimization.cpp src/transforms.cpp src/models.cpp src/util.cpp src/plyfile.c src/fitness.cpp src/testing.cpp)
target_link_libraries(${PROJECT_NAME} pcl_io)
rosbuild_add_openmp_flags(${PROJECT_NAME})

rosbuild_add_executable(compute_pcd_features src/compute_pcd_features.cpp)
target_link_libraries(compute_pcd_features ${PROJECT_NAME})

rosbuild_add_executable(bag_to_pcds src/bag_to_pcds.cpp)
#target_link_libraries(bag_to_pcds ${PROJECT_NAME})

#rosbuild_add_executable(test_transforms src/test_transforms.cpp)
#target_link_libraries(test_transforms ${PROJECT_NAME})

rosbuild_add_executable(test_hypothesizers src/test_hypothesizers.cpp)
target_link_libraries(test_hypothesizers ${PROJECT_NAME})


rosbuild_add_executable(object_detector src/object_detector.cpp)
target_link_libraries(object_detector ${PROJECT_NAME})
#rosbuild_add_compile_flags(object_detector "-pg")
#rosbuild_add_link_flags(object_detector "-pg")


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
