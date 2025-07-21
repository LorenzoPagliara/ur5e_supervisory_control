cmake_minimum_required(VERSION 3.8)
project(ur_force_torque_sensor)

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

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    rclcpp
    realtime_tools
    pluginlib
    ur_rtde
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# Shared library
add_library(ur_force_torque_sensor SHARED
            src/ur_force_torque_sensor.cpp
)

target_include_directories(
  ur_force_torque_sensor
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/ur_force_torque_sensor>
)

target_link_libraries(
  ur_force_torque_sensor
  PRIVATE ur_rtde::rtde
)

ament_target_dependencies(
  ur_force_torque_sensor
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface ur_force_torque_sensor.xml)

# Install
install(DIRECTORY include/ur_force_torque_sensor
        DESTINATION include/ur_force_torque_sensor
)

install(FILES ur_force_torque_sensor.xml
        DESTINATION share/ur_force_torque_sensor
)

install(TARGETS ur_force_torque_sensor
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

# Exports
ament_export_include_directories(include)
ament_export_libraries(ur_force_torque_sensor)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(ur_force_torque_sensor)

ament_package()
