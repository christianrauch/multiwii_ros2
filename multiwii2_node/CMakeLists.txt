cmake_minimum_required(VERSION 3.5)

project(multiwii_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(multiwii_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

find_package(PkgConfig)
pkg_check_modules(msp REQUIRED msp)
include_directories(${msp_INCLUDE_DIRS})
link_directories(${msp_LIBRARY_DIRS})

include_directories(include)

add_library(MultiWiiNode SHARED src/MultiWiiNode.cpp)
ament_target_dependencies(MultiWiiNode
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  multiwii_msgs
  mavros_msgs
  class_loader
  tf2_ros
)
target_link_libraries(MultiWiiNode ${msp_LIBRARIES})
rclcpp_components_register_nodes(MultiWiiNode "MultiWiiNode")

add_executable(multiwii_node src/multiwii_node.cpp)
target_link_libraries(multiwii_node MultiWiiNode)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS MultiWiiNode multiwii_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
