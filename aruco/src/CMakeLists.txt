add_library(
  bananas_aruco
  affine_rotation.cpp
  board.cpp
  box_board.cpp
  grid_board.cpp
  concrete_board.cpp
  world.cpp
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/affine_rotation.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/aruco_detector.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/board.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/box_board.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/grid_board.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/concrete_board.h"
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/world.h")

target_include_directories(bananas_aruco PUBLIC "${PROJECT_SOURCE_DIR}/include")

target_compile_features(bananas_aruco PUBLIC cxx_std_17)
set_target_properties(bananas_aruco PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(bananas_aruco PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(
  bananas_aruco PUBLIC Eigen3::Eigen ${OpenCV_LIBS} nlohmann_json::nlohmann_json
                      Microsoft.GSL::GSL)
if(WITH_NPP)
  target_compile_definitions(bananas_aruco PUBLIC ENABLE_CUSTOM_DETECTOR)
  target_link_libraries(bananas_aruco PUBLIC aruco_detector)
endif()

add_subdirectory(visualization)
