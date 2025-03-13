cmake_minimum_required(VERSION 3.17...3.31)
project(
  BananasArucoDetector
  VERSION 0.1
  DESCRIPTION "Modified version of OpenCV's ArUco detector using NPP"
  LANGUAGES CXX)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
  # OpenCV depends on the deprecated FindCUDA for some reason
  cmake_policy(SET CMP0146 OLD)
endif()

find_package(
  OpenCV
  CONFIG
  REQUIRED
  calib3d
  core
  highgui
  imgproc
  objdetect
  videoio)

find_package(CUDAToolkit REQUIRED)

add_subdirectory(src)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  add_subdirectory(apps)
endif()
