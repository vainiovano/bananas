add_executable(npp_testing npp_testing.cpp)

target_compile_features(npp_testing PUBLIC cxx_std_17)
set_target_properties(npp_testing PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(npp_testing PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  npp_testing
  PRIVATE CUDA::cudart
          CUDA::nppc
          CUDA::nppial
          CUDA::nppicc
          CUDA::nppif
          CUDA::nppitc
          CUDA::nppisu
          ${OpenCV_LIBS})
