cmake_minimum_required(VERSION 3.16)
project(task_space_mock_controller
        LANGUAGES CXX
)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    rclcpp
    geometry_msgs
    pluginlib
    hardware_interface
    acg_control_msgs
    acg_hardware_interface_facade
    acg_common_libraries
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(task_space_mock_controller_parameters
                           src/task_space_mock_controller_parameters.yaml
)

add_library(task_space_mock_controller SHARED
            src/task_space_mock_controller.cpp
)

target_compile_features(
  task_space_mock_controller
  PUBLIC cxx_std_17
)

target_include_directories(
  task_space_mock_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/task_space_mock_controller>
)

target_link_libraries(
  task_space_mock_controller
  PUBLIC task_space_mock_controller_parameters
)

ament_target_dependencies(
  task_space_mock_controller
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  task_space_mock_controller
  PRIVATE "TASK_SPACE_MOCK_CONTROLLER_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(controller_interface task_space_mock_controller.xml)

install(DIRECTORY include/
        DESTINATION include/task_space_mock_controller
)

install(TARGETS task_space_mock_controller task_space_mock_controller_parameters
        EXPORT export_task_space_mock_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_targets(export_task_space_mock_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
