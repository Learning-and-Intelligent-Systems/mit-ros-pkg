cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

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
find_package(wxWidgets COMPONENTS base core gl REQUIRED)
find_package(GLUT)
find_package(OpenGL)


configure_file( ${PROJECT_SOURCE_DIR}/src/imagepathconfig.h ${PROJECT_SOURCE_DIR}/src/imagepath.h)

add_definitions("`wx-config --cxxflags`")

FIND_PACKAGE(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories(${ImageMagick_INCLUDE_DIRS})

rosbuild_add_library(gatmounits src/gatmounits.cpp)

rosbuild_add_executable(calgui src/MRGui.cpp  src/globals.cpp src/MinorityReportDisplay.cpp src/RosConversion.cpp src/RosGlobals.cpp  src/FrameObject.cpp)
target_link_libraries(calgui gatmounits ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${wxWidgets_LIBRARIES}   ${ImageMagick_LIBRARIES} )

rosbuild_add_executable(testgui src/testopengl.cpp)
target_link_libraries(testgui ${GLUT_LIBRARY} ${OPENGL_LIBRARY}   ${ImageMagick_LIBRARIES}  )

