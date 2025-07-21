cmake_minimum_required(VERSION 3.8)
project(ur5e_app_screwdriving_supervisory_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
                  config
                  meshes
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
