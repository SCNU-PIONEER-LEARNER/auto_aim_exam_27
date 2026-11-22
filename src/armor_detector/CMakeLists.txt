## set OpenVINO_LIBRARIES
set(OpenVINO_LIBRARIES openvino::runtime)

cmake_minimum_required(VERSION 3.8)
project(armor_detector)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()
add_compile_options(-O1)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${OpenVINO_INCLUDE_DIRS} 
)

target_link_libraries(${PROJECT_NAME}
${OpenVINO_LIBRARIES} 
${OpenCV_LIBS}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_armor_pose_estimator test/test_armor_pose_estimator.cpp)
  target_link_libraries(test_armor_pose_estimator ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package()
