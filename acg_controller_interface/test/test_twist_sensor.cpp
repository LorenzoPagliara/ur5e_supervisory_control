/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_velocity_encoder.cpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    May 13, 2025
 *
 * This is a test file for the TwistSensor class.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ROS
#include <hardware_interface/loaned_state_interface.hpp>

// Class under test
#include "acg_semantic_components/twist_sensor.hpp"

class TwistSensorTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string sensor_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::TwistSensor> twist_sensor_;
};

void TwistSensorTest::SetUp()
{
  sensor_name_ = "test_twist_sensor";
}

void TwistSensorTest::TearDown()
{
  twist_sensor_.reset(nullptr);
}

/**
 * @brief Class that inherits from TwistSensorTest to have a common setup for tests
 */
class ConfiguredTwistSensorTest : public TwistSensorTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::vector<std::string> expected_state_interface_names_;
  std::size_t exp_size_{ 0 };
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces_;
};

void ConfiguredTwistSensorTest::SetUp()
{
  sensor_name_ = "test_twist_sensor";
  exp_size_ = 6;
  twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(sensor_name_, exp_size_);
  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

  expected_state_interface_names_ = {
    sensor_name_ + "/linear_velocity.x",  sensor_name_ + "/linear_velocity.y",  sensor_name_ + "/linear_velocity.z",
    sensor_name_ + "/angular_velocity.x", sensor_name_ + "/angular_velocity.y", sensor_name_ + "/angular_velocity.z"
  };

  state_interfaces_.reserve(exp_size_);
  loaned_state_interfaces_.reserve(exp_size_);
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    state_interfaces_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(sensor_name_, expected_state_interface_names_[i], &exp_values_[i]));
    loaned_state_interfaces_.emplace_back(*state_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the TwistSensor class with wrong size of interface names
 */
TEST_F(TwistSensorTest, test_twist_sensor_wrong_size_interface_names)
{
  // Create a vector with wrong size of interface names
  std::vector<std::string> custom_interface_names = { "test_twist_sensor/linear_velocity.x", "test_twist_sensor/linear_velocity.y" };

  // Expect an exception to be thrown
  EXPECT_THROW({ twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(sensor_name_, custom_interface_names); },
               std::invalid_argument);
}

/**
 * @brief Test the constructor of the TwistSensor class with custom interface names
 */
TEST_F(TwistSensorTest, test_twist_sensor_custom_interface_names)
{
  // Create a vector with custom interface names
  std::vector<std::string> custom_interface_names = { "test_twist_sensor/linear_velocity.x",  "test_twist_sensor/linear_velocity.y",
                                                      "test_twist_sensor/linear_velocity.z",  "test_twist_sensor/angular_velocity.x",
                                                      "test_twist_sensor/angular_velocity.y", "test_twist_sensor/angular_velocity.z" };

  // Create the TwistSensor object with custom interface names
  twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(sensor_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = twist_sensor_->get_state_interface_names();

  // Validate correct assignment of interface names
  ASSERT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the constructor of the TwistSensor class with default interface names
 */
TEST_F(ConfiguredTwistSensorTest, test_twist_sensor_default_interface_names)
{
  // Create the TwistSensor object with default interface names
  twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(sensor_name_);

  // Get the interface names
  interface_names_ = twist_sensor_->get_state_interface_names();

  // Validate correct assignment of interface names
  ASSERT_EQ(interface_names_.size(), expected_state_interface_names_.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_state_interface_names_.cbegin(),
                         expected_state_interface_names_.cend()));
}

/**
 * @brief Test the get_values_as_message method of the TwistSensor class
 */
TEST_F(ConfiguredTwistSensorTest, test_twist_sensor_get_values_as_message)
{
  // Create the TwistSensor object with default interface names
  twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(sensor_name_);

  // Get the values as message
  geometry_msgs::msg::Twist twist;
  EXPECT_TRUE(twist_sensor_->get_values_as_message(twist));

  // Validate the values
  EXPECT_DOUBLE_EQ(twist.linear.x, exp_values_[0]);
  EXPECT_DOUBLE_EQ(twist.linear.y, exp_values_[1]);
  EXPECT_DOUBLE_EQ(twist.linear.z, exp_values_[2]);
  EXPECT_DOUBLE_EQ(twist.angular.x, exp_values_[3]);
  EXPECT_DOUBLE_EQ(twist.angular.y, exp_values_[4]);
  EXPECT_DOUBLE_EQ(twist.angular.z, exp_values_[5]);
}
