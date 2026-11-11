cmake_minimum_required(VERSION 3.8)
project(virtual_low_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(virtual_low_board_node
  src/virtual_low_board_node.cpp
  src/crc.cpp
)
target_include_directories(virtual_low_board_node PRIVATE include)
ament_target_dependencies(virtual_low_board_node rclcpp std_msgs)

add_executable(packet_verifier_node
  src/packet_verifier_node.cpp
  src/crc.cpp
)
target_include_directories(packet_verifier_node PRIVATE include)
ament_target_dependencies(packet_verifier_node rclcpp std_msgs)

install(TARGETS
  virtual_low_board_node
  packet_verifier_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
