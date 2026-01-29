# Copyright (c) 2024，D-Robotics.
# 精简版：仅保留在线模式，SharedMem+NV12输入

cmake_minimum_required(VERSION 3.8)
project(face_landmarks_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 平台配置
if(PLATFORM_X3)
  message("=> build for platform X3")
  set(PREFIX_PATH x3)
elseif(PLATFORM_Rdkultra)
  message("=> build for platform Rdkultra")
  set(PREFIX_PATH rdkultra)
elseif(PLATFORM_X5)
  message("=> build for platform X5")
  set(PREFIX_PATH x5)
else()
  message("=> default: build for platform X5")
  set(PREFIX_PATH x5)
endif()

# 启用SharedMem
add_compile_definitions(SHARED_MEM_ENABLED)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("=> SYS_ROOT is " ${SYS_ROOT})

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dnn_node REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)

include_directories(${SYS_ROOT}/usr/include)
include_directories(${SYS_ROOT}/usr/hobot/include)
link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu
  ${SYS_ROOT}/usr/hobot/lib
  ${SYS_ROOT}/usr/lib
)

# 编译目标 (精简版：移除img_convert_utils和ai_msg_manage.cpp)
add_executable(face_landmarks_detection
  src/main.cpp
  src/face_landmarks_det_node.cpp
  src/face_landmarks_det_output_parser.cpp
)

target_compile_features(face_landmarks_detection PUBLIC c_std_99 cxx_std_17)
target_include_directories(face_landmarks_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(face_landmarks_detection dnn)

ament_target_dependencies(face_landmarks_detection
  rclcpp
  dnn_node
  ai_msgs
  hbm_img_msgs
)

# 安装
install(TARGETS face_landmarks_detection DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/${PREFIX_PATH}/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()
