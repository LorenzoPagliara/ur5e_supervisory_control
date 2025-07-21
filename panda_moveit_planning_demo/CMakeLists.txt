cmake_minimum_required(VERSION 3.22)
project(panda_moveit_planning_demo)

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
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(THIS_PACKAGE_DEPENDS
    ament_cmake
    rclcpp
    moveit_ros_planning_interface
    moveit_visual_tools
)

add_executable(panda_moveit_planning_node
               src/panda_moveit_planning_node.cpp
)
ament_target_dependencies(panda_moveit_planning_node ${THIS_PACKAGE_DEPENDS})

target_compile_features(
  panda_moveit_planning_node
  PUBLIC c_std_99 cxx_std_17
) # Require C99 and C++17

install(TARGETS panda_moveit_planning_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})

ament_package()
