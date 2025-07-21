cmake_minimum_required(VERSION 3.8)
project(moving_average_poses_filter)

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
    ament_cmake
    rclcpp
    filters
    pluginlib
    rcl_interfaces
    geometry_msgs
    ament_cmake_gtest
    Eigen3
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Shared library
add_library(moving_average_poses_filter SHARED
            src/moving_average_poses_filter.cpp
)

target_compile_features(
  moving_average_poses_filter
  PUBLIC cxx_std_17
)
# add_executable(parameters_node nodes/parameters_node.cpp )

# ament_target_dependencies(parameters_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(
  moving_average_poses_filter
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME})

ament_target_dependencies(
  moving_average_poses_filter
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(filters moving_average_poses_filter.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  moving_average_poses_filter
  PRIVATE "MOVING_AVERAGE_FILTER_BUILDING_LIBRARY"
)

# tests
if(BUILD_TESTING)
  # find dependencies
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(filters REQUIRED)

  set(THIS_TEST_PACKAGE_INCLUDE_DEPENDS
      ros_testing
      rclcpp
      filters
      pluginlib
      geometry_msgs
      Eigen3
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    find_package(${dependency} REQUIRED)
  endforeach()

  # add tests
  set(TEST_NAME
      ${PROJECT_NAME}_test
  )

  ament_add_gtest_executable(${TEST_NAME} test/test_moving_average_poses_filter.cpp)

  target_include_directories(
    ${TEST_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  add_ros_test(launch/test_moving_average_poses_filter.py)

  # ament_add_gtest(${TEST_NAME} test/test_moving_average_filter.cpp)

  target_link_libraries(${TEST_NAME} moving_average_poses_filter)

  # ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  # install test files
  install(DIRECTORY launch
          DESTINATION share/${PROJECT_NAME}
  )
  install(TARGETS ${TEST_NAME}
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY test
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# install(TARGETS parameters_node RUNTIME DESTINATION lib/${PROJECT_NAME} )

install(FILES moving_average_poses_filter.xml
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS moving_average_poses_filter
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(moving_average_poses_filter)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
