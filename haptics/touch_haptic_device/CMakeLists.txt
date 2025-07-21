cmake_minimum_required(VERSION 3.8)
project(touch_haptic_device)

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

find_path(OPENHAPTICS_INCLUDE_DIR
          NAMES HD/hd.h HDU/hdu.h
)
find_library(OPENHAPTICS_LIBRARY
             NAMES HD HDU
)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find OpenHaptics include directory")
endif()

if(NOT OPENHAPTICS_LIBRARY)
  message(FATAL_ERROR "Could not find OpenHaptics library")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    orocos_kdl
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include ${OPENHAPTICS_INCLUDE_DIR})

# Shared library
add_library(${PROJECT_NAME} SHARED
            src/touch_haptic_device.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC HD HDU
)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface touch_haptic_device.xml)

# Install
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
)

install(FILES touch_haptic_device.xml
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(${PROJECT_NAME})

ament_package()
