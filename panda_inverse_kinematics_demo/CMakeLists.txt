cmake_minimum_required(VERSION 3.22)
project(panda_inverse_kinematics_demo)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_geometry_msgs)
find_package(tf2_ros REQUIRED)

add_library(inverse_kinematics_demo SHARED
            src/inverse_kinematics_demo_node.cpp
)

ament_target_dependencies(
  inverse_kinematics_demo
  moveit_msgs
  moveit_ros_planning
  rclcpp
  rclcpp_components
  tf2_geometry_msgs
  tf2_ros
)

target_include_directories(
  inverse_kinematics_demo
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

target_compile_features(
  inverse_kinematics_demo
  PUBLIC c_std_99 cxx_std_17
) # Require C99 and C++17

rclcpp_components_register_node(
  inverse_kinematics_demo
  PLUGIN
  "panda_inverse_kinematics_demo::InverseKinematicsDemo"
  EXECUTABLE
  inverse_kinematics_demo_node
)

install(TARGETS inverse_kinematics_demo
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
