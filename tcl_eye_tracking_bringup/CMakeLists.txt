<<<<<<< HEAD
cmake_minimum_required(VERSION 3.8)
project(tcl_eye_tracking_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
=======
cmake_minimum_required(VERSION 3.8)
project(tcl_eye_tracking_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
>>>>>>> ab62ebc4a41d12306526d685243c840176a4c1c8
