add_library(
  aruco_detector aruco_detector.cpp aruco_utils.cpp aruco_utils.hpp
                 "${PROJECT_SOURCE_DIR}/include/bananas_aruco_detector/aruco_detector.hpp")

target_include_directories(aruco_detector PUBLIC "${PROJECT_SOURCE_DIR}/include")

target_compile_features(aruco_detector PUBLIC cxx_std_17)
set_target_properties(aruco_detector PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(aruco_detector PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(aruco_detector PUBLIC ${OpenCV_LIBS})
target_link_libraries(aruco_detector PRIVATE CUDA::cudart CUDA::nppc CUDA::nppif)
