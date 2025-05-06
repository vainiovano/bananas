macro(add_aruco_test TESTNAME)
  add_executable(${TESTNAME} ${ARGN})
  target_compile_features(${TESTNAME} PUBLIC cxx_std_17)
  set_target_properties(${TESTNAME} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${TESTNAME} PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${TESTNAME} GTest::gtest_main GTest::gmock
                        aruco_detector nlohmann_json::nlohmann_json)
  gtest_discover_tests(${TESTNAME} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endmacro()

add_aruco_test(box_board box_board.cpp)
add_aruco_test(grid_board grid_board.cpp)
add_aruco_test(mavlink mavlink.cpp)
