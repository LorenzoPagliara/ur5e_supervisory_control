cmake_minimum_required(VERSION 3.8)
project(twist_broadcaster)

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
    acg_controller_interface
    controller_interface
    generate_parameter_library
    geometry_msgs
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(twist_broadcaster_parameters src/twist_broadcaster_parameters.yaml)

# Shared library
add_library(${PROJECT_NAME} SHARED
            src/twist_broadcaster.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${PROJECT_NAME} twist_broadcaster_parameters)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugin
pluginlib_export_plugin_description_file(controller_interface twist_broadcaster.xml)

# Install
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
