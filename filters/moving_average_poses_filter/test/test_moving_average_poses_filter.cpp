/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_moving_average_poses_filter.cpp
 * Author:  Michele Marsico
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * This class provides a set of tests for the
 * MovingAveragePosesFilter class.
 *
 * -------------------------------------------------------------------
 */

#include <filters/filter_base.hpp>
#include "moving_average_poses_filter/moving_average_poses_filter.hpp"
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/SVD>

// Test fixture class for MovingAveragePosesFilter
class MovingAveragePosesFilterTester : public ::testing::Test
{
protected:
  // Called once before all tests
  static void SetUpTestCase() {}

  // Constructor: Initializes the filter loader and parameter node
  MovingAveragePosesFilterTester() : filter_loader_("filters", "filters::MultiChannelFilterBase<double>")
  {
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__node:=test_parameter_node" });
    parameter_node_ = std::make_shared<rclcpp::Node>("test_parameter_node", options);

    try
    {
      filter_ = filter_loader_.createSharedInstance("moving_average_poses_filter/MovingAveragePosesFilter");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("MovingAveragePosesFilterTester"), "The plugin failed to initialize. Error: %s", ex.what());
    }

    parameter_node_->declare_parameter("number_of_observations", 0);
  }

  // Destructor
  ~MovingAveragePosesFilterTester() {}

  // Called once after all tests
  static void TearDownTestCase() {}

  const double epsilon_ = 1e-6;           // Tolerance for floating point comparisons
  const uint8_t number_of_channels_ = 7;  // Number of channels in the filter
  bool configured_ = false;               // Flag to check if the filter is configured
  rclcpp::Logger logger_{ rclcpp::get_logger("MovingAveragePosesFilterTester") };
  std::shared_ptr<rclcpp::Node> parameter_node_;                                   // Node for parameter handling
  pluginlib::ClassLoader<filters::MultiChannelFilterBase<double>> filter_loader_;  // Plugin loader for the filter
  std::shared_ptr<filters::MultiChannelFilterBase<double>> filter_;                // Filter instance
};

// Function to compute the average pose from a circular buffer of poses
geometry_msgs::msg::Pose computeAveragePose(boost::circular_buffer<geometry_msgs::msg::Pose>& poses)
{
  geometry_msgs::msg::Pose average_pose;
  average_pose.position.x = average_pose.position.y = average_pose.position.z = 0;

  // Compute the average position
  for (std::size_t i = 0; i < poses.size(); i++)
  {
    average_pose.position.x += poses.at(i).position.x;
    average_pose.position.y += poses.at(i).position.y;
    average_pose.position.z += poses.at(i).position.z;
  }

  average_pose.position.x /= poses.size();
  average_pose.position.y /= poses.size();
  average_pose.position.z /= poses.size();

  /* Based on:

   F. L. Markley, Y. Cheng, J. L. Crassidis, and Y. Oshman.
   "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
   no. 4 (2007): 1193-1197. Link: https://ntrs.nasa.gov/citations/20070017872 */

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4, 4);

  // Compute the average orientation using quaternion averaging
  for (std::size_t i = 0; i < poses.size(); i++)
  {
    Eigen::Vector4d q(poses.at(i).orientation.x, poses.at(i).orientation.y, poses.at(i).orientation.z, poses.at(i).orientation.w);
    M += q * q.transpose();
  }

  M /= poses.size();

  // Select the eigenvector corresponding to the greatest eigenvalue of M
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd U = svd.matrixU();
  average_pose.orientation.x = U(0, 0);
  average_pose.orientation.y = U(1, 0);
  average_pose.orientation.z = U(2, 0);
  average_pose.orientation.w = U(3, 0);

  return average_pose;
}

// Function to convert a Pose message to a vector of doubles
std::vector<double> convertPoseToVector(geometry_msgs::msg::Pose pose)
{
  std::vector<double> pose_vector = { pose.position.x,    pose.position.y,    pose.position.z,   pose.orientation.x,
                                      pose.orientation.y, pose.orientation.z, pose.orientation.w };
  return pose_vector;
}

// Test case to check the configuration of the filter
TEST_F(MovingAveragePosesFilterTester, testConfigurationOnly)
{
  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  configured_ = filter_->configure(number_of_channels_, "", "MovingAveragePosesFilter", parameter_node_->get_node_logging_interface(),
                                   parameter_node_->get_node_parameters_interface());

  ASSERT_TRUE(configured_);
}

// Test case to check invalid configuration of the filter
TEST_F(MovingAveragePosesFilterTester, testInvalidConfiguration)
{
  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", 0));

  configured_ = filter_->configure(number_of_channels_, "", "MovingAveragePosesFilter", parameter_node_->get_node_logging_interface(),
                                   parameter_node_->get_node_parameters_interface());

  ASSERT_FALSE(configured_);
}

// Test case to check the filter with a two-sample window
TEST_F(MovingAveragePosesFilterTester, testTwoSamplesWindowDynamic)
{
  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  configured_ = filter_->configure(number_of_channels_, "", "MovingAveragePosesFilter", parameter_node_->get_node_logging_interface(),
                                   parameter_node_->get_node_parameters_interface());
  ASSERT_TRUE(configured_);

  std::vector<double> data_in = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
  std::vector<double> data_out(7);
  std::vector<double> expected_out = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };

  for (int i = 0; i < 2; i++)
  {
    filter_->update(data_in, data_out);
    ASSERT_NEAR(data_out[0], expected_out[0], epsilon_);
    ASSERT_NEAR(data_out[3], expected_out[3], epsilon_);
  }
}

// Test case to check the filter with a large window size
TEST_F(MovingAveragePosesFilterTester, testFiveThousandSamplesWindow)
{
  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  configured_ = filter_->configure(number_of_channels_, "", "MovingAveragePosesFilter", parameter_node_->get_node_logging_interface(),
                                   parameter_node_->get_node_parameters_interface());
  ASSERT_TRUE(configured_);

  int total_samples = 1e5;

  std::vector<double> pose_in = { 0, 0, 0.1, 0.001, 0, 0, 1 };
  std::vector<double> pose_out(number_of_channels_, 0);

  auto begin = std::chrono::steady_clock::now();

  for (int i = 0; i < total_samples; i++)
  {
    filter_->update(pose_in, pose_out);

    for (std::size_t j = 0; j < number_of_channels_; j++)
      ASSERT_NEAR(pose_in[j], pose_out[j], epsilon_);
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - begin;
  RCLCPP_INFO(logger_, "testFiveThousandSamplesWindow took %f s to complete.", elapsed_seconds.count());
}

// Test case to compare the custom filter output with the original filter output
TEST_F(MovingAveragePosesFilterTester, testCompareCustomAndOriginalFiltersOutput)
{
  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  configured_ = filter_->configure(number_of_channels_, "", "MovingAveragePosesFilter", parameter_node_->get_node_logging_interface(),
                                   parameter_node_->get_node_parameters_interface());
  ASSERT_TRUE(configured_);

  boost::circular_buffer<geometry_msgs::msg::Pose> poses;
  poses.set_capacity(2);

  geometry_msgs::msg::Pose first_pose;
  first_pose.position.x = 0.0;
  first_pose.position.y = 0.0;
  first_pose.position.z = 0.1;
  first_pose.orientation.x = 0.001;
  first_pose.orientation.y = 0.0;
  first_pose.orientation.z = 0.0;
  first_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose second_pose = first_pose;
  second_pose.position.z = 0.0;
  second_pose.orientation.x = 0.0;

  std::vector<double> first_pose_input_vector = convertPoseToVector(first_pose);
  std::vector<double> second_pose_input_vector = convertPoseToVector(second_pose);

  int total_samples = 1e5;

  std::vector<double> pose_in_vector = first_pose_input_vector;
  std::vector<double> pose_out_vector(number_of_channels_, 0);

  geometry_msgs::msg::Pose pose_in = first_pose;
  geometry_msgs::msg::Pose pose_out;

  geometry_msgs::msg::Pose average_pose;

  for (int i = 0; i < total_samples; i++)
  {
    if (i % 2 == 0)
    {
      pose_in_vector = first_pose_input_vector;
      pose_in = first_pose;
    }
    else
    {
      pose_in_vector = second_pose_input_vector;
      pose_in = second_pose;
    }

    filter_->update(pose_in_vector, pose_out_vector);

    poses.push_back(pose_in);
    average_pose = computeAveragePose(poses);
    std::vector<double> pose_out_vector_original = convertPoseToVector(average_pose);

    for (std::size_t j = 0; j < number_of_channels_; j++)
    {
      ASSERT_NEAR(pose_out_vector_original[j], pose_out_vector[j], epsilon_);
    }
  }
}

// Main function to run all tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
