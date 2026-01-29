# Copyright (c) 2024，D-Robotics.
# img_format_converter: mono8 -> nv12 格式转换节点

cmake_minimum_required(VERSION 3.8)
project(img_format_converter)

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

# 启用SharedMem
add_compile_definitions(SHARED_MEM_ENABLED)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("=> SYS_ROOT is " ${SYS_ROOT})

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)

include_directories(${SYS_ROOT}/usr/include)

# 编译目标
add_executable(img_format_converter
  src/main.cpp
  src/img_format_converter_node.cpp
)

target_compile_features(img_format_converter PUBLIC c_std_99 cxx_std_17)
target_include_directories(img_format_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(img_format_converter
  rclcpp
  sensor_msgs
  hbm_img_msgs
)

# 安装
install(TARGETS img_format_converter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
