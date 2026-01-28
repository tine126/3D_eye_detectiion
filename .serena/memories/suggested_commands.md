# 常用命令

## 构建命令
```bash
# 构建所有包
colcon build

# 构建单个包
colcon build --packages-select <package_name>

# 交叉编译 (RDK X5)
colcon build --cmake-args -DPLATFORM_X5=ON
```

## ROS2命令
```bash
# 查看话题列表
ros2 topic list

# 查看话题帧率
ros2 topic hz /topic_name

# 查看话题内容
ros2 topic echo /topic_name

# 启动节点
ros2 launch <package_name> <launch_file>.launch.py
```

## Windows系统命令
- dir - 列出目录
- type - 查看文件内容
- copy - 复制文件
- mkdir - 创建目录
