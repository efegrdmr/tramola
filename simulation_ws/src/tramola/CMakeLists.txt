cmake_minimum_required(VERSION 3.5)
project(tramola)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Detection.msg"
    "msg/DetectionList.msg"  # Add your message file(s) here
    DEPENDENCIES std_msgs
)

# Install Python modules and scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/followPath.py  # Adjust to the location of your Python script
    DESTINATION lib/${PROJECT_NAME}
)

# Add ament package
ament_package()
