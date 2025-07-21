cmake_minimum_required(VERSION 3.8)
project(haptic_ff_controller)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    hardware_interface
    pluginlib
    rclcpp_lifecycle
    geometry_msgs
    controller_interface
    moving_average_poses_filter
    filters
    generate_parameter_library
    Eigen3
    acg_control_msgs
    kinematics_interface
    pluginlib
    tf2
    tf2_geometry_msgs
    angles
    tf2_kdl
    tf2_eigen
    visualization_msgs
    realtime_tools
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(haptic_ff_controller_parameters # cmake target name for the parameter
                                                           # library
                           src/haptic_ff_controller_parameters.yaml # path to input yaml file
)

add_library(haptic_ff_controller SHARED
            src/haptic_ff_controller.cpp
)

target_compile_features(
  haptic_ff_controller
  PUBLIC cxx_std_17
)
target_include_directories(
  haptic_ff_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/haptic_ff_controller>
)
target_link_libraries(
  haptic_ff_controller
  PUBLIC haptic_ff_controller_parameters Eigen3::Eigen
)

include_directories(
  include
  ${moving_average_poses_filter_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# # Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when #
# building the dll but not consuming it.
target_compile_definitions(
  haptic_ff_controller
  PRIVATE "HAPTIC_FF_CONTROLLER_BUILDING_DLL"
)

ament_target_dependencies(
  haptic_ff_controller
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(controller_interface haptic_ff_controller.xml)

# Install
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(FILES haptic_ff_controller.xml
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS haptic_ff_controller haptic_ff_controller_parameters
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
