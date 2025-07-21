/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_admittance_filter.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    May 11, 2025
 *
 * Unit test for AdmittanceFilter class.
 * The ground truth is obtained with MATLAB by generating in Simulink
 * the expected output. Refer to the package Readme for more
 * information.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <acg_common_libraries/message_utilities.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <pluginlib/class_loader.hpp>
#include <interaction_filter_base/interaction_filter_base.hpp>
#include "admittance_filter/admittance_filter.hpp"

namespace interaction_filters
{

struct AdmittanceFilterParameters
{
  int filter_order;
  std::string input_type;
};

// This class shares parameters and data across all tests
class SharedData
{
  typedef pluginlib::ClassLoader<interaction_filters::InteractionFilterBase> FilterLoader;

  friend class AdmittanceFilterTest;

  // Private members
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<FilterLoader> filter_loader_;

  SharedData(const SharedData&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);

    // Instantiate the node
    rclcpp::NodeOptions node_options;
    node_options.context(context);
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("admittance_filter_test", node_options);

    node_->declare_parameter("admittance_filter.mass", std::vector<double>{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 });
    node_->declare_parameter("admittance_filter.damping_ratio", std::vector<double>{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 });
    node_->declare_parameter("admittance_filter.stiffness", std::vector<double>{ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 });
    node_->declare_parameter("admittance_filter.order", 2);
    node_->declare_parameter("admittance_filter.compliant_axis", std::vector<bool>{ true, true, true, true, true, true });

    // Initialize the filter class loader
    filter_loader_ = std::make_shared<pluginlib::ClassLoader<interaction_filters::InteractionFilterBase>>(
        "interaction_filter_base", "interaction_filters::InteractionFilterBase");
    ASSERT_TRUE(bool(filter_loader_)) << "Failed to instantiate ClassLoader<FilterLoader>";
  }

public:
  pluginlib::UniquePtr<interaction_filters::InteractionFilterBase> createUniqueInstance(const std::string& name) const
  {
    return filter_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.filter_loader_.reset();
  }
};

class AdmittanceFilterTest : public ::testing::TestWithParam<AdmittanceFilterParameters>
{
public:
  void SetUp() override;
  void TearDown() override
  {
    interaction_filter_.reset();
  }

  static void TearDownTestSuite()
  {
    // chiamata UNA sola volta alla fine di tutti i test
    interaction_filters::SharedData::release();
  }

protected:
  void operator=(const SharedData& data)
  {
    node_ = data.node_;
  }

  /**
   * @brief Shared pointer to the ROS2 node.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Input message for the filter.
   */
  std::vector<std_msgs::msg::Float64MultiArray> filter_input_;

  /**
   * @brief Output pose message of the filter.
   */
  std::vector<std_msgs::msg::Float64MultiArray> filter_output_pose_;

  /**
   * @brief Output twist message of the filter.
   */
  std::vector<std_msgs::msg::Float64MultiArray> filter_output_twist_;

  /**
   * @brief Output acceleration message of the filter.
   */
  std::vector<std_msgs::msg::Float64MultiArray> filter_output_acceleration_;

  /**
   * @brief Pointer to the instance of the admittance filter.
   */
  std::shared_ptr<interaction_filters::InteractionFilterBase> interaction_filter_;
};

void AdmittanceFilterTest::SetUp()
{
  *this = SharedData::instance();

  // Create the AdmittanceFilter instance
  ASSERT_NO_THROW({ interaction_filter_ = SharedData::instance().createUniqueInstance("interaction_filters/AdmittanceFilter"); });
  ASSERT_TRUE(bool(interaction_filter_)) << "Failed to load plugin: interaction_filters/AdmittanceFilter";

  // Initialize the parameters
  const AdmittanceFilterParameters& params = GetParam();

  // Set the parameters
  int filter_order = params.filter_order;
  node_->set_parameter(rclcpp::Parameter("admittance_filter.order", filter_order));
  node_->set_parameter(rclcpp::Parameter("admittance_filter.stiffness", std::vector<double>{ 900.0, 1300.0, 100.0, 100.0, 100.0, 100.0 }));
  node_->set_parameter(rclcpp::Parameter("admittance_filter.damping_ratio",
                                         std::vector<double>{ 1.58113883, 1.315587029, 1.58113883, 1.58113883, 1.58113883, 1.58113883 }));
  node_->set_parameter(rclcpp::Parameter("admittance_filter.mass", std::vector<double>{ 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 }));
  node_->set_parameter(rclcpp::Parameter("admittance_filter.compliant_axis", std::vector<bool>{ true, true, true, true, true, true }));

  // Initialize the filter
  interaction_filter_->initialize(node_->get_node_parameters_interface(), node_->get_node_logging_interface(), "admittance_filter");

  // Read trajectory from bag file
  std::string bag_filename = params.input_type + "_response_order_" + std::to_string(filter_order);
  std::string bagfile_path =
      ament_index_cpp::get_package_share_directory("admittance_filter") + "/test/bagfiles/" + bag_filename + "/" + bag_filename + "_0.db3";

  RCLCPP_INFO(node_->get_logger(), "Opening '%s'", bagfile_path.c_str());
  rosbag2_cpp::Reader reader;
  reader.open(bagfile_path);

  while (reader.has_next())
  {
    rosbag2_storage::SerializedBagMessageSharedPtr message = reader.read_next();

    if (message->topic_name == "/admittance_filter_input")
    {
      rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      std_msgs::msg::Float64MultiArray msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_input_.push_back(msg);
    }

    if (message->topic_name == "/admittance_filter_output/pose")
    {
      rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      std_msgs::msg::Float64MultiArray msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_output_pose_.push_back(msg);
    }

    if (message->topic_name == "/admittance_filter_output/twist")
    {
      rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      std_msgs::msg::Float64MultiArray msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_output_twist_.push_back(msg);
    }

    if (message->topic_name == "/admittance_filter_output/acceleration")
    {
      rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      std_msgs::msg::Float64MultiArray msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_output_acceleration_.push_back(msg);
    }
  }

  reader.close();
}

TEST_P(AdmittanceFilterTest, TestAdmittanceFilter)
{
  const double threshold = 1e-7;  // Tolerance for the test

  rclcpp::Duration previous_time = rclcpp::Duration(0, 0);
  for (size_t i = 0; i < filter_input_.size(); ++i)
  {
    // Set the filter input
    interaction_filters::Vector6d delta_h = Eigen::Map<interaction_filters::Vector6d>(filter_input_[i].data.data(), 6);

    rclcpp::Duration current_time = rclcpp::Duration::from_seconds(filter_input_[i].data[6]);

    // Set the reference for the admittance filter
    acg_control_msgs::msg::TaskSpacePoint admittance_filter_input;
    admittance_filter_input.pose = geometry_msgs::msg::Pose();
    admittance_filter_input.twist = geometry_msgs::msg::Twist();
    admittance_filter_input.acceleration = geometry_msgs::msg::Accel();
    tf2::toMsg(delta_h, admittance_filter_input.wrench);

    acg_control_msgs::msg::TaskSpacePoint admittance_filter_output;
    ASSERT_TRUE(interaction_filter_->update(admittance_filter_input, current_time - previous_time, admittance_filter_output));

    interaction_filters::Vector6d filter_output = Eigen::Map<interaction_filters::Vector6d>(filter_output_pose_[i].data.data(), 6);
    geometry_msgs::msg::Pose filter_output_pose;
    tf2::toMsg(filter_output, filter_output_pose);

    // Compare computed output with expected output (within a certain threshold)
    EXPECT_NEAR(admittance_filter_output.pose.position.x, filter_output_pose.position.x, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.position.y, filter_output_pose.position.y, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.position.z, filter_output_pose.position.z, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.x, filter_output_pose.orientation.x, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.y, filter_output_pose.orientation.y, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.z, filter_output_pose.orientation.z, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.w, filter_output_pose.orientation.w, threshold);
    EXPECT_NEAR(admittance_filter_output.twist.linear.x, filter_output_twist_[i].data[0], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.linear.y, filter_output_twist_[i].data[1], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.linear.z, filter_output_twist_[i].data[2], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.x, filter_output_twist_[i].data[3], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.y, filter_output_twist_[i].data[4], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.z, filter_output_twist_[i].data[5], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.x, filter_output_acceleration_[i].data[0], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.y, filter_output_acceleration_[i].data[1], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.z, filter_output_acceleration_[i].data[2], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.x, filter_output_acceleration_[i].data[3], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.y, filter_output_acceleration_[i].data[4], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.z, filter_output_acceleration_[i].data[5], threshold);

    previous_time = current_time;
  }
}

AdmittanceFilterParameters getAdmittanceFilterOrder0RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 0;
  params.input_type = "ramp";
  return params;
}

AdmittanceFilterParameters getAdmittanceFilterOrder1RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 1;
  params.input_type = "ramp";
  return params;
}

AdmittanceFilterParameters getAdmittanceFilterOrder2RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 2;
  params.input_type = "ramp";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder0StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 0;
  params.input_type = "step";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder1StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 1;
  params.input_type = "step";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder2StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 2;
  params.input_type = "step";
  return params;
}

INSTANTIATE_TEST_SUITE_P(AdmittanceFilterTests, AdmittanceFilterTest,
                         ::testing::Values(getAdmittanceFilterOrder0RampParameters(), getAdmittanceFilterOrder1RampParameters(),
                                           getAdmittanceFilterOrder2RampParameters(), getAdmittanceFilterOrder0StepParameters(),
                                           getAdmittanceFilterOrder1StepParameters(), getAdmittanceFilterOrder2StepParameters()));

}  // namespace interaction_filters
