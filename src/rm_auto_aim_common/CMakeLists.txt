cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} STATIC
  DIRECTORY src
)

# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)
# ament_export_dependencies(Eigen3 rclcpp rm_interfaces)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#############
## Install ##
#############


# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
# )

ament_auto_package(
  INSTALL_TO_SHARE
  include
)
