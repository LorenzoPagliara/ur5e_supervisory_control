cmake_minimum_required(VERSION 3.8)
project(endoscopic_camera_streaming_rviz)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher
               src/camera_publisher.cpp
)
ament_target_dependencies(
  camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS camera_publisher
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
