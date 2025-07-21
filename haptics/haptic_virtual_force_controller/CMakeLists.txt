cmake_minimum_required(VERSION 3.8)
project(haptic_virtual_force_controller)

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
    rclcpp
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    geometry_msgs
    controller_interface
    generate_parameter_library
    Eigen3
    control_toolbox
    realtime_tools
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(haptic_virtual_force_controller_parameters
                           src/haptic_virtual_force_controller_parameters.yaml
)

# add shered library
add_library(${PROJECT_NAME} SHARED
            src/haptic_virtual_force_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} haptic_virtual_force_controller_parameters)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugin
pluginlib_export_plugin_description_file(controller_interface haptic_virtual_force_controller.xml)

# Install
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(FILES haptic_virtual_force_controller.xml
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the line when a
  # copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND
      TRUE
  )
  # the following line skips cpplint (only works in a git repo) comment the line when this package
  # is in a git repo and when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND
      TRUE
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
