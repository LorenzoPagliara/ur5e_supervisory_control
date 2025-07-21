/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   camera_publisher.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Jul 9, 2025
 *
 * This module acquires images from a camera and publishes them
 * as ROS messages on the topic "camera/image_raw".
 * It uses OpenCV for image capture and cv_bridge for conversion
 * to ROS message format.
 * -------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher"), cap_(0, cv::CAP_V4L2)
  {
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Errore: impossibile aprire la camera /dev/video0");
      throw std::runtime_error("Failed to open camera");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Frame vuoto acquisito");
      return;
    }

    // Converti da BGR a RGB
    cv::Mat frame_rgb;
    cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);

    // Imposta header con timestamp e frame_id
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_frame";

    // Converti immagine OpenCV in messaggio ROS
    auto msg = cv_bridge::CvImage(header, "rgb8", frame_rgb).toImageMsg();

    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
