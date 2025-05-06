add_executable(generate_board generate_board.cpp)
target_compile_features(generate_board PUBLIC cxx_std_17)
set_target_properties(generate_board PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(generate_board PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(generate_board PRIVATE ${OpenCV_LIBS})

add_executable(positioner positioner.cpp)
target_compile_features(positioner PUBLIC cxx_std_17)
set_target_properties(positioner PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(positioner PRIVATE -Wall -Wextra -Wpedantic)
# NOTE: Can't use PRIVATE/PUBLIC here because ament_target_dependencies doesn't
# support them.
target_link_libraries(positioner ${OpenCV_LIBS} aruco_detector aruco_visualizer
                      Microsoft.GSL::GSL MAVSDK::mavsdk)
if(WITH_ROS2)
  target_compile_definitions(positioner PRIVATE ENABLE_ROS2)
  ament_target_dependencies(positioner rclcpp std_msgs sensor_msgs cv_bridge
                            image_transport)
endif()

if(WITH_ROS2)
  add_executable(ros_video_publisher ros_video_publisher.cpp)
  target_compile_features(ros_video_publisher PUBLIC cxx_std_17)
  set_target_properties(ros_video_publisher PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(ros_video_publisher PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(ros_video_publisher PRIVATE ENABLE_ROS2)
  # NOTE: Can't use PRIVATE/PUBLIC here because ament_target_dependencies
  # doesn't support them.
  target_link_libraries(ros_video_publisher ${OpenCV_LIBS})
  ament_target_dependencies(ros_video_publisher rclcpp std_msgs sensor_msgs
                            cv_bridge image_transport)
endif()

add_executable(gltf_exporter gltf_exporter.cpp)
target_compile_features(gltf_exporter PUBLIC cxx_std_17)
set_target_properties(gltf_exporter PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(gltf_exporter PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  gltf_exporter
  PRIVATE ${OpenCV_LIBS} aruco_detector tinygltf Microsoft.GSL::GSL
          nlohmann_json::nlohmann_json tinyxml2::tinyxml2)
