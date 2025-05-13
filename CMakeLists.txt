cmake_minimum_required(VERSION 3.8)
project(dynamixel_rdk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
  /home/eungang/ros2_ws/install/dynamixel_sdk_custom_interfaces/include
)

# -------------------------------
# BaseSettingNode는 라이브러리로 빌드
# -------------------------------
add_library(base_setting_node
  src/base_setting_node.cpp
)

target_include_directories(base_setting_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(base_setting_node
  rclcpp
  dynamixel_sdk
)

# -------------------------------
# 실행 파일들
# -------------------------------
add_executable(motor_control_node src/motor_control_node.cpp)
add_executable(motor_status_node src/motor_status_node.cpp)
add_executable(dynamixel_communication_node src/dynamixel_communication_node.cpp)

# -------------------------------
# 링크 설정
# -------------------------------
target_link_libraries(motor_control_node
  base_setting_node
  ${dynamixel_sdk_LIBRARIES}
)

target_link_libraries(motor_status_node
  base_setting_node
  ${dynamixel_sdk_LIBRARIES}
)

target_link_libraries(dynamixel_communication_node
  base_setting_node
  ${dynamixel_sdk_LIBRARIES}
)

# -------------------------------
# ament dependencies
# -------------------------------
ament_target_dependencies(motor_control_node
  rclcpp
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

ament_target_dependencies(motor_status_node
  rclcpp
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

ament_target_dependencies(dynamixel_communication_node
  rclcpp
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)

# -------------------------------
# Install targets
# -------------------------------
install(TARGETS
  motor_control_node
  motor_status_node
  dynamixel_communication_node
  base_setting_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# -------------------------------
# Testing
# -------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
