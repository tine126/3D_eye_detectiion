# Copyright (c) 2024，D-Robotics.
# eye_position_publisher: 从人脸关键点提取眼睛中心坐标

cmake_minimum_required(VERSION 3.8)
project(eye_position_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 平台配置
if(PLATFORM_X3)
  message("=> build for platform X3")
elseif(PLATFORM_Rdkultra)
  message("=> build for platform Rdkultra")
elseif(PLATFORM_X5)
  message("=> build for platform X5")
else()
  message("=> default: build for platform X5")
endif()

set(SYS_ROOT ${CMAKE_SYSROOT})
message("=> SYS_ROOT is " ${SYS_ROOT})

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EyePositions.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(${SYS_ROOT}/usr/include)

# 编译目标
add_executable(eye_position_publisher_node
  src/main.cpp
  src/eye_position_publisher_node.cpp
)

target_compile_features(eye_position_publisher_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(eye_position_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(eye_position_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  ai_msgs
)

# 链接自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(eye_position_publisher_node "${cpp_typesupport_target}")

# 安装
install(TARGETS eye_position_publisher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
