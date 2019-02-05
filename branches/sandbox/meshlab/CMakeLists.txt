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

set(CMAKE_CXX_FLAGS  "-I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4/QtXml -I/usr/include/qt4/QtScript -I/usr/include/qt4 -I${PROJECT_SOURCE_DIR}/src -pipe -O2 -Wall -W -D_REENTRANT -fPIC -DQT_NO_DEBUG -DQT_PLUGIN -DQT_SCRIPT_LIB -DQT_XML_LIB -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -L/usr/lib -L/usr/X11R6/lib -L../../distrib -lQtScript -lQtXml -lQtOpenGL -lQtGui -lQtCore -lGLU -lGL -lpthread")

set(CMAKE_C_FLAGS "-I${PROJECT_SOURCE_DIR}/src")

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

rosbuild_add_library(ply2stl src/ply2stl.cpp src/io_base/baseio.cpp src/wrap/ply/plylib.cpp src/io_base/moc_baseio.cpp src/common/filterparameter.cpp src/common/meshmodel.cpp src/common/interfaces.cpp src/common/moc_meshmodel.cpp src/common/GLLogStream.cpp src/GL/glew.c)

rosbuild_add_executable(mesh_convert src/mesh_convert.cpp)
target_link_libraries(mesh_convert ply2stl)