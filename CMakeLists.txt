cmake_minimum_required(VERSION 3.8)
project(dynamixel_rdk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)

set(EXECUTABLES
  dynamixel_rdk_ros2_node
)

foreach(executable ${EXECUTABLES})
  add_executable(${executable} 
    src/dynamixel_rdk_ros2.cpp
    src/motor_status.cpp
    src/motor_setting.cpp
  )

  target_include_directories(${executable} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${executable}
    rclcpp
    dynamixel_sdk
    dynamixel_rdk_msgs
  )
endforeach()

install(TARGETS
  ${EXECUTABLES}
  # base_setting_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# config 디렉토리가 실제로 존재할 때만 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# -------------------------------
# Testing
# -------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()