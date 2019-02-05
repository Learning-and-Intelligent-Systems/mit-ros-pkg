cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_CXX_FLAGS  "-fpermissive")
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)


###rosbuild_add_library(mesher src/cloud_to_mesh.cpp)
###target_link_libraries(mesher ply2stl Poisson )



#-----------------------------------------------
#----commented out while I change message types!
rosbuild_add_executable(surveyer src/surveyer.cpp)

###rosbuild_add_library(scananalyzer src/ScanAnalyzer.cpp)
###target_link_libraries(scananalyzer pcl_utils mesher)

###rosbuild_add_executable(analyzer src/object_analysis.cpp)
###target_link_libraries(analyzer scananalyzer)
###rosbuild_add_compile_flags(analyzer " -fopenmp ")

###rosbuild_add_executable(analyzer2 src/object_analysis2.cpp)
###target_link_libraries(analyzer2 scananalyzer)
###rosbuild_add_compile_flags(analyzer2 " -fopenmp ")



#program to do normalization, find sift features and save as scanAnalysis messages
###rosbuild_add_executable(initial_processing src/initialProcessing.cpp)
###target_link_libraries(initial_processing scananalyzer)

#-----------------------------------------------


rosbuild_add_executable(filterFeatures src/filterFeatures.cpp)


rosbuild_add_executable(setprojector src/dynamic_recon.cpp)



rosbuild_add_executable(camera_surveyer src/camera_surveyer.cpp)
rosbuild_add_executable(testparallel src/testparallel.cpp)
target_link_libraries(testparallel pcl_utils)



#---------------------- JGLOV -------------------------

rosbuild_add_executable(publish_stereo_clouds src/publish_stereo_clouds.cpp)

