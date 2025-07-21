cmake_minimum_required(VERSION 3.8)
project(haptic_fb_controller)

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
find_package(ament_cmake
             REQUIRED
             touch_haptic_device
             REQUIRED
             COMPONENTS
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    acg_control_msgs
    rclcpp
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    geometry_msgs
    controller_interface
    generate_parameter_library
    filters
    kinematics_interface
    Eigen3
    realtime_tools
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(haptic_fb_controller_parameters src/haptic_fb_controller_parameters.yaml)

# include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# add shered library
add_library(haptic_fb_controller SHARED
            src/haptic_fb_controller.cpp
)

target_include_directories(
  haptic_fb_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/haptic_fb_controller>
)

target_link_libraries(
  haptic_fb_controller
  haptic_fb_controller_parameters
  Eigen3::Eigen
)

ament_target_dependencies(haptic_fb_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugin
pluginlib_export_plugin_description_file(controller_interface haptic_fb_controller.xml)

# Install
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(FILES haptic_fb_controller.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS haptic_fb_controller haptic_fb_controller_parameters
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
