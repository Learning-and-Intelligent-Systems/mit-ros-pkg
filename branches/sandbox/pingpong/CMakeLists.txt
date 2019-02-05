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
rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#rosbuild_add_boost_directories()

# add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

rosbuild_add_executable(eyes src/eyes.cpp src/util.cpp src/optimization.cpp)
rosbuild_add_compile_flags(eyes "-O3 -msse2 -msse3")
#rosbuild_add_compile_flags(eyes "-pg")
#rosbuild_add_link_flags(eyes "-pg")
#rosbuild_add_openmp_flags(eyes)
target_link_libraries(eyes bingham lapacke lapack blas gfortran)

rosbuild_add_executable(track_player src/track_player.cpp)
target_link_libraries(track_player bingham lapacke lapack blas gfortran)

rosbuild_add_executable(find_table src/find_table.cpp src/util.cpp src/optimization.cpp)
rosbuild_add_compile_flags(find_table "-O3 -msse2 -msse3")
target_link_libraries(find_table bingham lapacke lapack blas gfortran)

rosbuild_add_executable(draw_eyes src/draw_eyes.cpp src/util.cpp src/optimization.cpp)
rosbuild_add_compile_flags(draw_eyes "-O3 -msse2 -msse3")
target_link_libraries(draw_eyes bingham lapacke lapack blas gfortran)

rosbuild_add_executable(brain src/brain.cpp src/kinematics.cpp src/dynamics.cpp)
rosbuild_add_compile_flags(brain "-O3 -msse2 -msse3")
target_link_libraries(brain bingham lapacke lapack blas gfortran)
#rosbuild_add_compile_flags(brain "-pg")
#rosbuild_add_link_flags(brain "-pg")

rosbuild_add_executable(test_ik src/test_ik.cpp src/kinematics.cpp src/dynamics.cpp src/optimization.cpp)
target_link_libraries(test_ik bingham lapacke lapack blas gfortran)

rosbuild_add_executable(train_swing src/train_swing.cpp src/kinematics.cpp src/dynamics.cpp src/optimization.cpp)
target_link_libraries(train_swing bingham lapacke lapack blas gfortran)

rosbuild_add_executable(swing_arm src/swing_arm.cpp src/dynamics.cpp)
target_link_libraries(swing_arm bingham lapacke lapack blas gfortran)

rosbuild_add_executable(hallucinate_ball src/hallucinate_ball.cpp)
target_link_libraries(hallucinate_ball bingham lapacke lapack blas gfortran)

rosbuild_add_executable(record_ball_trajectories src/record_ball_trajectories.cpp)
target_link_libraries(record_ball_trajectories bingham lapacke lapack blas gfortran)

rosbuild_add_executable(record_swing_and_ball_trajectories src/record_swing_and_ball_trajectories.cpp)
target_link_libraries(record_swing_and_ball_trajectories bingham lapacke lapack blas gfortran)

rosbuild_add_executable(send_hit_policies src/send_hit_policies.cpp)
target_link_libraries(send_hit_policies bingham lapacke lapack blas gfortran)

rosbuild_add_executable(send_hit_plan src/send_hit_plan.cpp)
target_link_libraries(send_hit_plan bingham lapacke lapack blas gfortran)

rosbuild_add_executable(reload_swing_table src/reload_swing_table.cpp)



#rosbuild_add_executable(extract_images src/extract_images.cpp)
#rosbuild_add_executable(extract_eyes_state src/extract_eyes_state.cpp)

#rosbuild_add_executable(simtest src/simtest.cpp)
#rosbuild_add_compile_flags(simtest "-O3 -msse2 -msse3")
#rosbuild_add_compile_flags(simtest "-g")
#rosbuild_add_compile_flags(simtest "-pg")
#rosbuild_add_link_flags(simtest "-pg")
#target_link_libraries(simtest bingham flann lapacke lapack blas gfortran)

#rosbuild_add_executable(test_dynamics src/test_dynamics.cpp src/dynamics.cpp)
#rosbuild_add_compile_flags(test_dynamics "-O3 -msse2 -msse3")
#rosbuild_add_compile_flags(test_dynamics "-g -O0")
#rosbuild_add_compile_flags(test_dynamics "-pg")
#rosbuild_add_link_flags(test_dynamics "-pg")
#target_link_libraries(test_dynamics bingham flann lapacke)
#target_link_libraries(test_dynamics bingham flann lapacke lapack blas gfortran)
#rosbuild_link_boost(test_dynamics signals thread)

#rosbuild_add_executable(optimizer src/optimizer.cpp src/dynamics.cpp)
#rosbuild_add_compile_flags(optimizer)
#target_link_libraries(optimizer bingham flann lapacke lapack blas gfortran)

#rosbuild_add_executable(test_optimizer src/test_optimizer.cpp src/dynamics.cpp src/optimizer.cpp)
#rosbuild_add_compile_flags(test_optimizer)
#target_link_libraries(test_optimizer bingham lapacke lapack blas gfortran)


#rosbuild_add_executable(ball_detector src/ball_detector.cpp src/optimization.cpp src/util.cpp)
#rosbuild_add_compile_flags(ball_detector "-O3 -msse2 -msse3")
#target_link_libraries(ball_detector ${PCL_LIBRARIES})

#rosbuild_add_executable(table_detector src/table_detector.cpp src/optimization.cpp src/util.cpp)
#rosbuild_add_compile_flags(table_detector "-O3 -msse2 -msse3")
#target_link_libraries(table_detector bingham ${PCL_LIBRARIES})



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
