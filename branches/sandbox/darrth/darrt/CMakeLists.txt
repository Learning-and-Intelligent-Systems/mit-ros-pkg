cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

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

#rosbuild_add_library(simple_pr2_kinematics_lib
#  src/simple_pr2_kinematics_plugin.cpp
#)

 
#have removed for the moment
#src/astar_solver.cpp



rosbuild_add_library(darrt_lib
  src/darrth.cpp
  src/object_solver.cpp
  src/object_types.cpp
  src/object_space.cpp
  src/pr2_arm_primitives.cpp
  src/pr2_base_primitives.cpp
  src/space.cpp
  src/goal.cpp
  src/solver.cpp
  src/object_primitives.cpp
  src/environment_interface.cpp
  src/robot_base_space.cpp
  src/robot_space.cpp
  src/chain_space.cpp
  src/ompl_ros_conversions.cpp
  src/control.cpp
  src/state_transformer.cpp
  src/support_surface.cpp
  src/transform_ros_types.cpp
  src/collision_aware_types.cpp
  src/utils.cpp
  src/primitive.cpp
)

rosbuild_add_executable(spatula src/spatula_test.cpp)
target_link_libraries(spatula darrt_lib)
target_link_libraries(spatula ${OMPL_LIBRARIES})

rosbuild_add_executable(collision_test src/collision_test.cpp)
target_link_libraries(collision_test darrt_lib)
target_link_libraries(collision_test ${OMPL_LIBRARIES})

# rosbuild_add_executable(table_main src/table_main.cpp)
# target_link_libraries(table_main darrt_lib)
# target_link_libraries(table_main ${OMPL_LIBRARIES})


#target_link_libraries(sp_test darrt_lib)

#rosbuild_add_executable(base_pusher src/base_pushing_main.cpp)
#target_link_libraries(base_pusher pr2_manipulation_lib)
#rosbuild_add_executable(chain_pusher src/chain_pushing_main.cpp)
#target_link_libraries(chain_pusher pr2_manipulation_lib)

#rosbuild_add_executable(pr2_alone src/simple_pr2_ompl_planner.cpp)
#rosbuild_add_executable(planning_scene_test src/planning_scene_client_test.cpp)
