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

# -------------------------------
# BaseSettingNode 라이브러리
# -------------------------------
# add_library(base_setting_node
#   src/base_setting_node.cpp
# )

# target_include_directories(base_setting_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(
#   base_setting_node
#   rclcpp
#   dynamixel_sdk
#   dynamixel_sdk_custom_interfaces
# )

# -------------------------------
# 실행 파일들
# -------------------------------

# EXECUTABLES 리스트를 먼저 정의
set(EXECUTABLES
  # motor_control_node
  # motor_status_node
  # dynamixel_communication_node
  dynamixel_rdk_ros2
)

# 각 실행 파일 생성
foreach(executable ${EXECUTABLES})
  add_executable(${executable} src/${executable}.cpp)

  # target_link_libraries(${executable}
  #   base_setting_node
  # )

  target_include_directories(${executable} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${executable}
    rclcpp
    dynamixel_sdk
    dynamixel_sdk_custom_interfaces
  )
endforeach()

# -------------------------------
# Install targets
# -------------------------------
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
