cmake_minimum_required(VERSION 3.10)
project(rm_tracker)

## optimize while compiling
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.

add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ARMOR_TRACKER_USE_IESKF "Use iterative error-state Kalman filter in armor tracker" OFF)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

file(GLOB_RECURSE RM_TRACKER_SOURCES CONFIGURE_DEPENDS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
    ${RM_TRACKER_SOURCES}
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC ARMOR_TRACKER_USE_IESKF=$<BOOL:${ARMOR_TRACKER_USE_IESKF}>
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN rm_auto_aim::tracker::TrackerNode
    EXECUTABLE ${PROJECT_NAME}_node
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

    ament_add_gtest(test_armor_overlay test/test_armor_overlay.cpp)
    target_link_libraries(test_armor_overlay ${PROJECT_NAME})

    ament_add_gtest(test_robot_target_geometry test/test_robot_target_geometry.cpp)
    target_link_libraries(test_robot_target_geometry ${PROJECT_NAME})
endif()

#############
## Install ##
#############

ament_auto_package()
