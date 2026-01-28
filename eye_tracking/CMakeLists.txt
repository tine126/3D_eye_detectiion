cmake_minimum_required(VERSION 3.5)
project(eye_tracking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(eye_tracking_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Combined Node (all nodes in one executable)
add_executable(eye_tracking_node
  src/main.cpp
  src/eye_position_2d_node.cpp
  src/eye_position_3d_node.cpp
  src/eye_visualization_node.cpp
)
ament_target_dependencies(eye_tracking_node
  rclcpp sensor_msgs ai_msgs eye_tracking_msgs
  message_filters cv_bridge OpenCV
)

# Main Node (simplified single-file version)
add_executable(eye_tracking_main
  src/eye_tracking_main.cpp
)
ament_target_dependencies(eye_tracking_main
  rclcpp ai_msgs message_filters
)

install(TARGETS
  eye_tracking_node
  eye_tracking_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
