/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   touch_haptic_device.cpp
 * Author:  Antonio Langella, Michele Marsico, Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 1, 2024
 *
 * This module implements the HapticDeviceTouch class, which is a
 * ROS2 hardware interface for the Touch Haptic Device.
 *
 * -------------------------------------------------------------------
 */

#include "touch_haptic_device/touch_haptic_device.hpp"

#include <random>
#include <pluginlib/class_list_macros.hpp>
#include <stdio.h>
#include <kdl/frames.hpp>
#include <limits>
#include <HD/hdDefines.h>
#include <HDU/hduMatrix.h>

namespace haptic_device
{

// Static member initialization
HHD HapticDeviceTouch::haptic_device_handler_ = HD_INVALID_HANDLE;

HapticDeviceTouch::HapticDeviceTouch() {}

HapticDeviceTouch::~HapticDeviceTouch() {}

hardware_interface::CallbackReturn HapticDeviceTouch::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(*logger_, "Configuring the device");

  // Reset joint states during configuration
  std::fill(touch_haptic_device_states_.begin(), touch_haptic_device_states_.end(), 0.0);

  // Reset joint commands during configuration
  std::fill(touch_haptic_device_commands_.begin(), touch_haptic_device_commands_.end(), 0.0);

  // Initialize the last haptic pose
  last_haptic_pose_.position.x = 0.0;
  last_haptic_pose_.position.y = 0.0;
  last_haptic_pose_.position.z = 0.0;
  last_haptic_pose_.orientation.x = 0.0;
  last_haptic_pose_.orientation.y = 0.0;
  last_haptic_pose_.orientation.z = 0.0;
  last_haptic_pose_.orientation.w = 0.0;

  rendered_force_.x = 0.0;
  rendered_force_.y = 0.0;
  rendered_force_.z = 0.0;

  // Initialize the device
  enable_device_();
  wait_for_stable_pose_readings_();

  message_flag_ = false;

  RCLCPP_INFO(*logger_, "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(*logger_, "Activating the device");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(*logger_, "Deactivating the device");

  // Disable the device
  disable_device_();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(*logger_, "Cleaning up the device");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(*logger_, "Shutting down the device");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_ERROR(*logger_, "Error in the device");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HapticDeviceTouch::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get logger
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger("TouchHapticDevice"));

  // Initialize state and command interfaces
  touch_haptic_device_states_.resize(info_.gpios[0].state_interfaces.size());
  touch_haptic_device_commands_.resize(info_.gpios[0].command_interfaces.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HapticDeviceTouch::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < touch_haptic_device_states_.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.gpios[0].name, info_.gpios[0].state_interfaces[i].name, &touch_haptic_device_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HapticDeviceTouch::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < touch_haptic_device_commands_.size(); i++)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.gpios[0].name, info_.gpios[0].command_interfaces[i].name, &touch_haptic_device_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type HapticDeviceTouch::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
  geometry_msgs::msg::Pose haptic_device_pose;

  // Ensure that the getHapticDevicePose function is available and correctly implemented
  if (!get_haptic_device_pose_(haptic_device_pose))
  {
    RCLCPP_ERROR(*logger_, "Failed to read the haptic device pose");
    return hardware_interface::return_type::ERROR;
  }

  // Compute the twist from the haptic device pose
  KDL::Frame haptic_frame = KDL::Frame(KDL::Rotation::Quaternion(haptic_device_pose.orientation.x, haptic_device_pose.orientation.y,
                                                                 haptic_device_pose.orientation.z, haptic_device_pose.orientation.w),
                                       KDL::Vector(haptic_device_pose.position.x, haptic_device_pose.position.y, haptic_device_pose.position.z));

  KDL::Frame last_haptic_frame = KDL::Frame(KDL::Rotation::Quaternion(last_haptic_pose_.orientation.x, last_haptic_pose_.orientation.y,
                                                                      last_haptic_pose_.orientation.z, last_haptic_pose_.orientation.w),
                                            KDL::Vector(last_haptic_pose_.position.x, last_haptic_pose_.position.y, last_haptic_pose_.position.z));

  KDL::Twist haptic_twist_kdl = KDL::diff(last_haptic_frame, haptic_frame, period.seconds());

  // Update the last haptic pose
  last_haptic_pose_ = haptic_device_pose;

  // Update haptic device states with the read values
  touch_haptic_device_states_[0] = haptic_device_pose.position.x;
  touch_haptic_device_states_[1] = haptic_device_pose.position.y;
  touch_haptic_device_states_[2] = haptic_device_pose.position.z;
  touch_haptic_device_states_[3] = haptic_device_pose.orientation.x;
  touch_haptic_device_states_[4] = haptic_device_pose.orientation.y;
  touch_haptic_device_states_[5] = haptic_device_pose.orientation.z;
  touch_haptic_device_states_[6] = haptic_device_pose.orientation.w;

  touch_haptic_device_states_[7] = haptic_twist_kdl.vel[0];
  touch_haptic_device_states_[8] = haptic_twist_kdl.vel[1];
  touch_haptic_device_states_[9] = haptic_twist_kdl.vel[2];
  touch_haptic_device_states_[10] = haptic_twist_kdl.rot[0];
  touch_haptic_device_states_[11] = haptic_twist_kdl.rot[1];
  touch_haptic_device_states_[12] = haptic_twist_kdl.rot[2];

  RCLCPP_DEBUG(rclcpp::get_logger("HapticDeviceTouch"),
               "Updated state interfaces with pose values: "
               "Position: (%.2f, %.2f, %.2f), Orientation: (%.2f, %.2f, %.2f, %.2f)",
               touch_haptic_device_states_[0], touch_haptic_device_states_[1], touch_haptic_device_states_[2], touch_haptic_device_states_[3],
               touch_haptic_device_states_[4], touch_haptic_device_states_[5], touch_haptic_device_states_[6]);

  RCLCPP_DEBUG(rclcpp::get_logger("HapticDeviceTouch"),
               "Updated state interfaces with twist values: "
               "Linear: (%.2f, %.2f, %.2f), Angular: (%.2f, %.2f, %.2f)",
               touch_haptic_device_states_[7], touch_haptic_device_states_[8], touch_haptic_device_states_[9], touch_haptic_device_states_[10],
               touch_haptic_device_states_[11], touch_haptic_device_states_[12]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HapticDeviceTouch::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (touch_haptic_device_commands_.size() < 3)
  {
    RCLCPP_ERROR(*logger_, "Insufficient command data");
    return hardware_interface::return_type::ERROR;
  }

  rendered_force_.x = touch_haptic_device_commands_[0];
  rendered_force_.y = touch_haptic_device_commands_[1];
  rendered_force_.z = touch_haptic_device_commands_[2];

  if (!get_inkwell_switch_state_(inkwell_switch_))
  {
    RCLCPP_ERROR(*logger_, "Failed to read the inkwell switch state");
    return hardware_interface::return_type::ERROR;
  }

  if (inkwell_switch_ == HD_TRUE || (rendered_force_.x == 0.0 && rendered_force_.y == 0.0 && rendered_force_.z == 0.0))
  {
    if (!set_haptic_device_cartesian_forces_(rendered_force_))
    {
      RCLCPP_ERROR(*logger_, "Failed to write the haptic device forces");
      return hardware_interface::return_type::ERROR;
    }
    message_flag_ = false;
  }
  else
  {
    if (!message_flag_)
    {
      RCLCPP_INFO(*logger_, "Please remove the stylus from the inkwell");
      message_flag_ = true;
    }
    rendered_force_.x = 0.0;
    rendered_force_.y = 0.0;
    rendered_force_.z = 0.0;
    if (!set_haptic_device_cartesian_forces_(rendered_force_))
    {
      RCLCPP_ERROR(*logger_, "Failed to write the haptic device forces");
      return hardware_interface::return_type::ERROR;
    }
  }

  RCLCPP_DEBUG(*logger_, "Updated command interfaces with force values: : x=%.5f, y=%.5f, z=%.5f", rendered_force_.x, rendered_force_.y,
               rendered_force_.z);

  return hardware_interface::return_type::OK;
}

bool HapticDeviceTouch::get_haptic_device_pose_(geometry_msgs::msg::Pose& haptic_device_pose) const
{
  hdScheduleSynchronous(HapticDeviceTouch::get_haptic_device_pose_callback_, &haptic_device_pose, HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in get_haptic_device_pose_callback: ");
  return true;
}

bool HapticDeviceTouch::get_haptic_device_cartesian_angles_(geometry_msgs::msg::Vector3& haptic_device_cartesian_angles) const
{
  hdScheduleSynchronous(HapticDeviceTouch::get_haptic_device_cartesian_angles_callback_, &haptic_device_cartesian_angles,
                        HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in get_haptic_device_cartesian_angles_callback: ");
  return true;
}

bool HapticDeviceTouch::get_haptic_device_cartesian_velocity_(geometry_msgs::msg::Vector3& haptic_device_cartesian_velocity) const
{
  hdScheduleSynchronous(HapticDeviceTouch::get_haptic_device_cartesian_velocity_callback_, &haptic_device_cartesian_velocity,
                        HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in get_haptic_device_cartesian_velocity_callback: ");
  return true;
}

bool HapticDeviceTouch::set_haptic_device_cartesian_forces_(const geometry_msgs::msg::Vector3& forces) const
{
  HDfloat cartesian_forces[3];
  cartesian_forces[0] = forces.x;
  cartesian_forces[1] = forces.y;
  cartesian_forces[2] = forces.z;
  hdScheduleAsynchronous(HapticDeviceTouch::set_haptic_device_cartesian_forces_callback_, cartesian_forces, HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in set_hatic_device_cartesian_forces_callback: ");
  return true;
}

bool HapticDeviceTouch::set_haptic_device_joint_torques_(const geometry_msgs::msg::Vector3& torques) const
{
  HDfloat joint_torques[3];
  joint_torques[0] = torques.x;
  joint_torques[1] = torques.y;
  joint_torques[2] = torques.z;
  hdScheduleAsynchronous(HapticDeviceTouch::set_haptic_device_joint_torques_callback_, joint_torques, HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in set_haptic_device_joint_torques_callback: ");
  return true;
}

bool HapticDeviceTouch::set_haptic_device_cartesian_torques_(const geometry_msgs::msg::Vector3&) const
{
  throw haptic_device::Exception("This device doesn't have the ability to set the cartesian torques");
}

bool HapticDeviceTouch::get_inkwell_switch_state_(HDboolean& inkwell_switch) const
{
  hdScheduleSynchronous(HapticDeviceTouch::get_inkwell_switch_state_callback_, &inkwell_switch, HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in get_inkwell_switch_state_callback: ");
  return true;
}

bool HapticDeviceTouch::get_haptic_device_angular_velocity_(geometry_msgs::msg::Vector3& haptic_device_angular_velocity) const
{
  hdScheduleSynchronous(HapticDeviceTouch::get_haptic_device_angular_velocity_callback_, &haptic_device_angular_velocity,
                        HD_DEFAULT_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in get_haptic_device_angular_velocity_callback: ");
  return true;
}

bool HapticDeviceTouch::get_update_rate_(int& update_rate)
{
  hdScheduleSynchronous(HapticDeviceTouch::get_update_rate_callback_, &update_rate, HD_MAX_SCHEDULER_PRIORITY);
  check_haptic_device_error_("HDError in getUpdateRateCallback: ");
  return true;
}

void HapticDeviceTouch::enable_device_()
{
  haptic_device_handler_ = hdInitDevice(HD_DEFAULT_DEVICE);
  RCLCPP_INFO(*logger_, "Current device handle: %x", haptic_device_handler_);
  // enable forces/torques output
  if (!hdIsEnabled(HD_FORCE_OUTPUT))
  {
    hdEnable(HD_FORCE_OUTPUT);
  }

  HDErrorInfo error;
  // check the initialization result
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    throw haptic_device::Exception("Device Initialization Failed");
  }

  // start the scheduler, which manages callbacks to be executed within the servo loop thread
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    throw haptic_device::Exception("Scheduler failed to start");
  }

  // calibrate device
  if (!calibrate_())
  {
    throw haptic_device::Exception("Calibration failed");
  }
}

void HapticDeviceTouch::disable_device_() const
{
  hdStopScheduler();
  hdDisableDevice(haptic_device_handler_);
}

void HapticDeviceTouch::wait_for_stable_pose_readings_() const
{
  geometry_msgs::msg::Pose haptic_device_pose_previous;
  geometry_msgs::msg::Pose haptic_device_pose_current;
  get_haptic_device_pose_(haptic_device_pose_previous);

  unsigned int stable_readings = 0;

  while (stable_readings < MIN_NUMBER_OF_STABLE_READINGS)
  {
    get_haptic_device_pose_(haptic_device_pose_current);

    if (haptic_device_pose_previous == haptic_device_pose_current)
      stable_readings++;
    else
      stable_readings = 0;

    haptic_device_pose_previous = haptic_device_pose_current;
  }
}

bool HapticDeviceTouch::calibrate_() const
{
  RCLCPP_INFO(*logger_, "Starting touch haptic device calibration");

  HDenum calibration_status;

  if (!get_calibration_status_(calibration_status))
    return false;

  if (calibration_status != HD_CALIBRATION_OK)
    RCLCPP_INFO(*logger_, "Please place the stylus into the inkwell");

  while (calibration_status != HD_CALIBRATION_OK)
    if (!get_calibration_status_(calibration_status))
      return false;

  RCLCPP_INFO(*logger_, "Device calibrated");

  return true;
}

HDCallbackCode HapticDeviceTouch::set_haptic_device_cartesian_forces_callback_(void* user_data)
{
  HDfloat* cartesian_forces = (HDfloat*)user_data;
  hdBeginFrame(haptic_device_handler_);
  hdSetFloatv(HD_CURRENT_FORCE, cartesian_forces);
  hdEndFrame(haptic_device_handler_);
  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::set_haptic_device_joint_torques_callback_(void* user_data)
{
  HDfloat* joint_torques = (HDfloat*)user_data;
  hdBeginFrame(haptic_device_handler_);
  hdSetFloatv(HD_CURRENT_JOINT_TORQUE, joint_torques);
  hdEndFrame(haptic_device_handler_);
  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::calibration_status_callback_(void* user_data)
{
  HDenum* calibration_status = (HDenum*)user_data;

  hdBeginFrame(haptic_device_handler_);

  *calibration_status = hdCheckCalibration();

  hdEndFrame(haptic_device_handler_);

  // check if the calibration needs to be updated.
  if (*calibration_status == HD_CALIBRATION_NEEDS_UPDATE)
  {
    RCLCPP_INFO(rclcpp::get_logger("HapticDeviceTouch"), "Updating calibration");
    HDint style;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &style);
    // update the calibration
    hdUpdateCalibration(style);
  }

  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_inkwell_switch_state_callback_(void* user_data)
{
  HDboolean* inkwell_switch = (HDboolean*)user_data;

  hdBeginFrame(haptic_device_handler_);

  hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, inkwell_switch);

  hdEndFrame(haptic_device_handler_);
  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_haptic_device_pose_callback_(void* user_data)
{
  geometry_msgs::msg::Pose* haptic_device_pose = (geometry_msgs::msg::Pose*)user_data;
  hdBeginFrame(haptic_device_handler_);
  // get transformation matrix from base frame to end-effector frame
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdEndFrame(haptic_device_handler_);
  // get rotation matrix
  hduMatrix hd_rotation_matrix;

  transform.getRotationMatrix(hd_rotation_matrix);
  tf2::Matrix3x3 tf2_rotation_matrix(transform[0][0], transform[1][0], transform[2][0], transform[0][1], transform[1][1], transform[2][1],
                                     transform[0][2], transform[1][2], transform[2][2]);
  // set position vector
  double mm_to_m = 1000;
  haptic_device_pose->position.x = transform[3][0] / mm_to_m;
  haptic_device_pose->position.y = transform[3][1] / mm_to_m;
  haptic_device_pose->position.z = transform[3][2] / mm_to_m;

  // set orientation
  double roll, pitch, yaw;
  tf2_rotation_matrix.getRPY(roll, pitch, yaw);
  tf2::Quaternion tf2_quaternion;
  tf2_quaternion.setRPY(roll, pitch, yaw);
  haptic_device_pose->orientation = tf2::toMsg(tf2_quaternion);

  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_haptic_device_cartesian_angles_callback_(void* user_data)
{
  geometry_msgs::msg::Vector3* haptic_device_cartesian_angles = (geometry_msgs::msg::Vector3*)user_data;

  hdBeginFrame(haptic_device_handler_);

  hduVector3Dd angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, angles);
  hdEndFrame(haptic_device_handler_);

  haptic_device_cartesian_angles->x = angles[0];
  haptic_device_cartesian_angles->y = angles[1];
  haptic_device_cartesian_angles->z = angles[2];

  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_haptic_device_cartesian_velocity_callback_(void* user_data)
{
  geometry_msgs::msg::Vector3* haptic_device_cartesian_velocity = (geometry_msgs::msg::Vector3*)user_data;

  hdBeginFrame(haptic_device_handler_);
  HDfloat velocity[3];
  hdGetFloatv(HD_CURRENT_VELOCITY, velocity);
  hdEndFrame(haptic_device_handler_);

  haptic_device_cartesian_velocity->x = velocity[0] / 1000;
  haptic_device_cartesian_velocity->y = velocity[1] / 1000;
  haptic_device_cartesian_velocity->z = velocity[2] / 1000;

  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_haptic_device_angular_velocity_callback_(void* user_data)
{
  geometry_msgs::msg::Vector3* haptic_device_angular_velocity = (geometry_msgs::msg::Vector3*)user_data;

  hdBeginFrame(haptic_device_handler_);
  HDfloat angular_velocity[3];
  hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, angular_velocity);
  hdEndFrame(haptic_device_handler_);

  haptic_device_angular_velocity->x = angular_velocity[0];
  haptic_device_angular_velocity->y = angular_velocity[1];
  haptic_device_angular_velocity->z = angular_velocity[2];

  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_last_haptic_device_error_callback_(void* user_data)
{
  HDErrorInfo* error = (HDErrorInfo*)user_data;
  // get the last error in the error stack
  *error = hdGetError();
  return HD_CALLBACK_DONE;
}

HDCallbackCode HapticDeviceTouch::get_update_rate_callback_(void* user_data)
{
  int* update_rate = (int*)user_data;
  hdBeginFrame(haptic_device_handler_);
  hdGetIntegerv(HD_INSTANTANEOUS_UPDATE_RATE, update_rate);
  hdEndFrame(haptic_device_handler_);
  return HD_CALLBACK_DONE;
}

bool HapticDeviceTouch::get_calibration_status_(HDenum& calibration_status)
{
  hdScheduleSynchronous(HapticDeviceTouch::calibration_status_callback_, &calibration_status, HD_DEFAULT_SCHEDULER_PRIORITY);

  check_haptic_device_error_("HDError in calibrationStatusCallback: ");
  return true;
}

void HapticDeviceTouch::check_haptic_device_error_(std::string error_message)
{
  HDErrorInfo error = get_last_haptic_device_error_();
  if (HD_DEVICE_ERROR(error))
  {
    std::ostringstream stream;
    stream << error_message << hdGetErrorString(error.errorCode);
    throw haptic_device::Exception(stream.str());
  }
}

HDErrorInfo HapticDeviceTouch::get_last_haptic_device_error_()
{
  HDErrorInfo error;
  hdScheduleSynchronous(HapticDeviceTouch::get_last_haptic_device_error_callback_, &error, HD_DEFAULT_SCHEDULER_PRIORITY);
  return error;
}

}  // namespace haptic_device

PLUGINLIB_EXPORT_CLASS(haptic_device::HapticDeviceTouch, hardware_interface::SystemInterface)
