add_library(
  aruco_visualizer
  visualizer.cpp
  "${PROJECT_SOURCE_DIR}/include/bananas_aruco/visualization/visualizer.h")

target_include_directories(aruco_visualizer
                           PUBLIC "${PROJECT_SOURCE_DIR}/include")

target_compile_features(aruco_visualizer PUBLIC cxx_std_17)
set_target_properties(aruco_visualizer PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(aruco_visualizer PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(aruco_visualizer PUBLIC bananas_aruco OgreBites
                                              Microsoft.GSL::GSL)
