cmake_minimum_required(VERSION 3.8)
project(dynamixel_rdk_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentMotorStatus.msg"
  "msg/DynamixelControlMsgs.msg"
  "msg/DynamixelMsgs.msg"
  "msg/WarningStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
