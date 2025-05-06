cmake_minimum_required(VERSION 3.16...4.0)
project(
  BananasArucoDetector
  VERSION 1.0
  DESCRIPTION "Try to find boxes using ArUco markers"
  LANGUAGES CXX)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
  # OpenCV depends on the deprecated FindCUDA for some reason
  cmake_policy(SET CMP0146 OLD)
endif()

include(FetchContent)
FetchContent_Declare(
  GSL
  URL "https://github.com/microsoft/GSL/archive/87f9d768866548b5b86e72be66c60c5abd4d9b37.zip"
)
FetchContent_MakeAvailable(GSL)

FetchContent_Declare(
  tinygltf
  URL "https://github.com/syoyo/tinygltf/archive/a5e653e46cca64bbc0ecee7ede007347a94c39b0.zip"
)
set(TINYGLTF_BUILD_LOADER_EXAMPLE
    OFF
    CACHE BOOL "")
set(TINYGLTF_HEADER_ONLY
    ON
    CACHE BOOL "")
set(TINYGLTF_INSTALL
    OFF
    CACHE BOOL "")
set(TINYGLTF_INSTALL_VENDOR
    OFF
    CACHE BOOL "")
FetchContent_MakeAvailable(tinygltf)

FetchContent_Declare(
  googletest
  URL "https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip"
)

find_package(nlohmann_json 3.11.3 REQUIRED CONFIG)

find_package(tinyxml2 REQUIRED CONFIG)

find_package(Eigen3 3.4 REQUIRED CONFIG)

find_package(
  OpenCV
  CONFIG
  REQUIRED
  core
  objdetect
  videoio
  highgui
  calib3d
  imgcodecs)

find_package(OGRE REQUIRED COMPONENTS Bites CONFIG)

find_package(MAVSDK REQUIRED CONFIG)

option(WITH_ROS2 "Enable ROS2 support" OFF)
if(WITH_ROS2)
  find_package(ament_cmake CONFIG REQUIRED)
  find_package(rclcpp CONFIG REQUIRED)
  find_package(std_msgs CONFIG REQUIRED)
  find_package(sensor_msgs CONFIG REQUIRED)
  find_package(cv_bridge CONFIG REQUIRED)
  find_package(image_transport CONFIG REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(gazebo)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
  if(BUILD_TESTING)
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)
    add_subdirectory(tests)
  endif()
endif()
