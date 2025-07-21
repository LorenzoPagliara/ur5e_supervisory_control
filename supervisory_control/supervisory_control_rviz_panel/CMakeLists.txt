cmake_minimum_required(VERSION 3.8)
project(supervisory_control_rviz_panel)

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
find_package(ament_cmake_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    pluginlib
    rviz_common
    supervisory_control_rviz_msg
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

set(CMAKE_AUTOMOC
    ON
)
qt5_wrap_cpp(MOC_FILES include/supervisory_control_rviz_panel/supervisory_control_rviz_panel.hpp)

add_library(supervisory_control_rviz_panel
            src/supervisory_control_rviz_panel.cpp ${MOC_FILES}
)

target_include_directories(
  supervisory_control_rviz_panel
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/supervisory_control_rviz_panel>
)

ament_target_dependencies(supervisory_control_rviz_panel ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

install(TARGETS supervisory_control_rviz_panel
        EXPORT export_supervisory_control_rviz_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY icons/classes/
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
