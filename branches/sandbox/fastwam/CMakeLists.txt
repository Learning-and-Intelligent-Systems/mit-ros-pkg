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


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


# Location of custom Finders
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")

## Xenomai
#find_package(Xenomai REQUIRED)
find_package(Xenomai)

if ( Xenomai_FOUND )
#if ( XENOMAI_DIR )

  message("Found Xenomai\n")

  include_directories(${XENOMAI_INCLUDE_DIR})
  add_definitions(${XENOMAI_DEFINITIONS})

  ## GSL
  find_package(GSL REQUIRED)
  include_directories(${GSL_INCLUDE_DIRS})

  ## Boost
  set(Boost_ADDITIONAL_VERSIONS "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0" "1.59" "1.59.0")
  find_package(Boost 1.45.0 REQUIRED thread)
  include_directories(${Boost_INCLUDE_DIRS})

  ## Eigen2
  find_package(Eigen2 REQUIRED)
  include_directories(${EIGEN2_INCLUDE_DIR})

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_executable(arm src/arm.cpp src/util.cpp src/kinematics.cpp)
  rosbuild_add_compile_flags(arm "-DEIGEN2_SUPPORT")
  target_link_libraries(arm barrett bingham ${GSL_LIBRARIES} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE} lapacke lapack blas gfortran)
  
  rosbuild_add_executable(torque_test src/torque_test.cpp)
  rosbuild_add_executable(swingdata_logger src/swingdata_logger.cpp)


  # add dynamic reconfigure api
  #rosbuild_find_ros_package(dynamic_reconfigure)
  #include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
  #gencfg()

  #rosbuild_add_executable(wamtest src/wamtest.cpp)
  #rosbuild_add_compile_flags(wamtest "-DEIGEN2_SUPPORT")
  #rosbuild_add_compile_flags(wamtest "-O3 -msse2 -msse3 -ffast-math")
  #rosbuild_add_compile_flags(wamtest "-pg")
  #rosbuild_add_link_flags(wamtest "-pg")
  #rosbuild_add_openmp_flags(wamtest)
  #target_link_libraries(wamtest barrett ${GSL_LIBRARIES} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE})

  #rosbuild_add_executable(safety_controller src/safety_controller.cpp src/util.cpp)
  #rosbuild_add_compile_flags(safety_controller "-DEIGEN2_SUPPORT")
  #target_link_libraries(safety_controller barrett ${GSL_LIBRARIES} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE})

  #rosbuild_add_executable(swingtest src/swingtest.cpp src/util.cpp)
  #rosbuild_add_compile_flags(swingtest "-DEIGEN2_SUPPORT")
  #target_link_libraries(swingtest barrett ${GSL_LIBRARIES} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE})

  #add_definitions("-DROSCONSOLE_SEVERITY_INFO")
  #rosbuild_add_executable(wamtest_client src/wamtest_client.cpp)
  #target_link_libraries(wamtest_client barrett ${GSL_LIBRARIES} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE})

endif ( Xenomai_FOUND )
#endif ( XENOMAI_DIR )



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
