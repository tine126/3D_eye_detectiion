# Copyright (c) 2024，D-Robotics.
# 精简版：只保留人脸检测功能

cmake_minimum_required(VERSION 3.5)
project(mono2d_body_detection)

# C99 / C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hbm_img_msgs REQUIRED)

# 启用SharedMem
add_definitions(-DSHARED_MEM_ENABLED)

include_directories(include ${PROJECT_SOURCE_DIR})

# 平台配置
if(PLATFORM_X3)
  message("Build for platform X3")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH x3)
elseif(PLATFORM_Rdkultra)
  message("Build for platform Rdkultra")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH rdkultra)
elseif(PLATFORM_X5)
  message("Build for platform X5")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH x5)
else()
  message("Default: Build for platform X5")
  add_definitions(-DBPU_LIBDNN)
  set(PREFIX_PATH x5)
  set(SYS_ROOT ${CMAKE_SYSROOT})
endif()

message("CMAKE_SYSROOT: ${CMAKE_SYSROOT}")

include_directories(
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/hobot/include/
  ${SYS_ROOT}/usr/include/hobot/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/hobot/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
)

# 编译目标 (精简版：移除yolo_pose_parser)
add_library(${PROJECT_NAME}_component SHARED
  src/main.cpp
  src/mono2d_body_det_node.cpp
)

target_compile_definitions(${PROJECT_NAME}_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(
  ${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  dnn_node
  ai_msgs
  hobot_cv
  hbm_img_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "Mono2dBodyDetNode"
  EXECUTABLE ${PROJECT_NAME}
)

# 安装
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_component
  DESTINATION lib/
)

# 安装模型文件
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/${PREFIX_PATH}/
  DESTINATION lib/${PROJECT_NAME}/config/
)

# 安装launch文件
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
