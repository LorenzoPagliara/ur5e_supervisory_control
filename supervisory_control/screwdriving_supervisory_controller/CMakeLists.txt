cmake_minimum_required(VERSION 3.8)
project(screwdriving_supervisory_controller)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    acg_control_msgs
    rclcpp
    moveit_ros_planning_interface
    moveit_visual_tools
    controller_manager_msgs
    supervisory_control_rviz_msg
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(screwdriving_supervisory_controller
               src/screwdriving_supervisory_control_node.cpp
)
ament_target_dependencies(screwdriving_supervisory_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_features(
  screwdriving_supervisory_controller
  PUBLIC c_std_99 cxx_std_17
) # Require C99 and C++17

install(TARGETS screwdriving_supervisory_controller
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
