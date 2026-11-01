cmake_minimum_required(VERSION 3.5)
project(rm_tools)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs)
find_package(OpenCV 4 REQUIRED)
find_package(rcpputils REQUIRED)


# include
include_directories(include)

# create rm_utils lib
add_library(${PROJECT_NAME} SHARED
  src/math.cpp
  src/pnp_solver.cpp
  src/extended_kalman_filter.cpp
  src/ieskf.cpp
  src/manual_compensator.cpp
  src/trajectory_compensator.cpp
  src/rotation_bridge.cpp
  src/image_projection.cpp
  src/heartbeat.cpp
)

set(dependencies
  rclcpp
  rcpputils
  Eigen3
  eigen3_cmake_module
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  OpenCV
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export lib lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} fmt)

# test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pnp_solver test/test_pnp_solver.cpp)
  target_link_libraries(test_pnp_solver ${PROJECT_NAME})

  ament_add_gtest(test_debug_gate test/test_debug_gate.cpp)
  target_link_libraries(test_debug_gate ${PROJECT_NAME})

  ament_add_gtest(test_thread_safe_queue test/test_thread_safe_queue.cpp)
  target_link_libraries(test_thread_safe_queue ${PROJECT_NAME})

  ament_add_gtest(test_thread_pool test/test_thread_pool.cpp)
  target_link_libraries(test_thread_pool ${PROJECT_NAME})

  ament_add_gtest(test_iterated_extended_kalman_filter test/test_iterated_extended_kalman_filter.cpp)
  target_link_libraries(test_iterated_extended_kalman_filter ${PROJECT_NAME})

  ament_add_gtest(test_image_projection test/test_image_projection.cpp)
  target_link_libraries(test_image_projection ${PROJECT_NAME})

  ament_add_gtest(test_image_utils test/test_image_utils.cpp)
  target_link_libraries(test_image_utils ${PROJECT_NAME})

  ament_add_gtest(test_rune_geometry test/test_rune_geometry.cpp)
  target_link_libraries(test_rune_geometry ${PROJECT_NAME})

  ament_add_gtest(test_heartbeat test/test_heartbeat.cpp)
  target_link_libraries(test_heartbeat ${PROJECT_NAME})
endif()

ament_package()
