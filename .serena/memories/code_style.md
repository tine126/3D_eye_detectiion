# 代码风格规范

## 命名规范
- 类名: PascalCase (如 Mono2dBodyDetNode)
- 成员变量: snake_case_ 带下划线后缀 (如 model_file_name_)
- 方法名: PascalCase (如 RosImgProcess)
- 常量: snake_case 或 kCamelCase

## 代码风格
- C++14标准
- 2空格缩进
- 使用智能指针 (std::shared_ptr)
- ROS2节点继承自DnnNode基类
- 使用rclcpp组件注册方式

## 文件组织
- include/包名/ - 头文件
- src/ - 源文件
- launch/ - 启动文件
- config/ - 配置文件
