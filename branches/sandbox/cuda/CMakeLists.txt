cmake_minimum_required(VERSION 2.6.2)

project(Cuda-project)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/cuda" ${CMAKE_MODULE_PATH})

# find_package(CUDA)
find_package(CUDA QUIET REQUIRED)

##############################################################################
# There are two ways to add include directories to the NVCC command
# line:

#set(CUDA_PROPAGATE_HOST_FLAGS OFF)

# The cuda_include_directories adds paths to only cuda compilation.
CUDA_INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

# The include_directories adds paths to both C/C++ compilation in the native
# compiler and cuda compilation in NVCC.  Note that CUDA_INCLUDE_DIRS is added
# automatically by CUDA_ADD_EXECUTABLE and CUDA_ADD_LIBRARY.

# INCLUDE_DIRECTORIES(
#   ${CUDA_INCLUDE_DIRS}
#   )

##############################################################################
# There are four ways to compile source files with NVCC. 

# Set CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE when you want to add the same .cu
# file to multiple targets.
set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)


message(STATUS "cuda include dirs: ${CUDA_INCLUDE_DIRS}")

CUDA_ADD_EXECUTABLE(vectortest
  vectorAdd.cu
  )


CUDA_BUILD_CLEAN_TARGET()

