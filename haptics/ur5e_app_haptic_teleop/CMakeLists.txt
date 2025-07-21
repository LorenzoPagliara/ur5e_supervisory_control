cmake_minimum_required(VERSION 3.8)
project(ur5e_app_haptic_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
