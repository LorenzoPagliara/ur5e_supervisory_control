/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_poses_filter.cpp
 * Author:  Michele Marsico
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include <moving_average_poses_filter/moving_average_poses_filter.hpp>
#include <Eigen/SVD>
#include <rclcpp/rclcpp.hpp>

using namespace moving_average_poses_filter;

template <typename T>
MovingAveragePosesFilter<T>::MovingAveragePosesFilter() : number_of_position_channels_(3), number_of_orientation_channels_(4)
{}

template <typename T>
MovingAveragePosesFilter<T>::~MovingAveragePosesFilter()
{}

template <typename T>
bool MovingAveragePosesFilter<T>::configure()
{
  if (!MovingAveragePosesFilter<T>::getParam("number_of_observations", number_of_observations_))
  {
    RCLCPP_ERROR(logger_, "Could not retrieve 'number_of_observations' from parameter server.");
    return false;
  }

  if (number_of_observations_ < 1)
  {
    RCLCPP_ERROR(logger_, "Parameter 'number_of_observations' should be greater than or equal to 1.");
    return false;
  }

  if (number_of_channels_ != (number_of_position_channels_ + number_of_orientation_channels_))
  {
    RCLCPP_ERROR(logger_, "The number of channels should be equal to 7: 3 for the position and 4 for the orientation");
    return false;
  }

  // Resize helper variables for the right number of channels
  removed_position_.resize(number_of_position_channels_, 0);
  position_cumulator_.resize(number_of_position_channels_, 0);
  position_zeros_.resize(number_of_position_channels_, 0);

  removed_orientation_.setZero();
  orientation_cumulator_.setZero();
  orientation_zeros_.setZero();

  // Delete any object that was previously pointed to by the storages
  positions_storage_.reset(new filters::RealtimeCircularBuffer<std::vector<T>>(number_of_observations_, position_zeros_));

  std::vector<T> orientation_vector_zeros(number_of_orientation_channels_ * number_of_orientation_channels_, 0.0);
  orientations_storage_.reset(new filters::RealtimeCircularBuffer<std::vector<T>>(number_of_observations_, orientation_vector_zeros));

  return true;
}

template <typename T>
bool MovingAveragePosesFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
  if (!configured_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MovingAveragePosesFilter"), "The filter must be configured before using the update() method.");
    return false;
  }

  if ((data_in.size() != number_of_channels_) || (data_out.size() != number_of_channels_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MovingAveragePosesFilter"), "Input and output parameters with sizes %lu and %lu differ from configuration: %lu.",
                 data_in.size(), data_out.size(), number_of_channels_);
    return false;
  }

  // Check if an element needs to be removed from storages
  if (positions_storage_->size() != number_of_observations_)
    removed_position_ = position_zeros_;
  else
    removed_position_ = positions_storage_->front();

  if (orientations_storage_->size() != number_of_observations_)
    removed_orientation_ = orientation_zeros_;
  else
    removed_orientation_ =
        Eigen::Map<Matrix4x<T>>(orientations_storage_->front().data(), number_of_orientation_channels_, number_of_orientation_channels_);

  // Create vectors for the input position and for the input orientation
  std::vector<T> position_in(data_in.begin(), data_in.begin() + number_of_position_channels_);
  std::vector<T> quaternion_in(data_in.begin() + number_of_position_channels_, data_in.end());

  // Add new position to storage
  positions_storage_->push_back(position_in);

  // Update cumulators and output for each channel of the position
  for (std::size_t i = 0; i < position_in.size(); i++)
  {
    position_cumulator_[i] = position_cumulator_[i] - removed_position_[i] + position_in[i];
    data_out[i] = position_cumulator_[i] / positions_storage_->size();
  }

  /* Based on:

   F. L. Markley, Y. Cheng, J. L. Crassidis, and Y. Oshman.
   "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
   no. 4 (2007): 1193-1197. Link: https://ntrs.nasa.gov/citations/20070017872 */

  Vector4x<T> q(quaternion_in[0], quaternion_in[1], quaternion_in[2], quaternion_in[3]);

  Matrix4x<T> orientation_term = q * q.transpose();
  std::vector<T> orientation_in(orientation_term.data(), orientation_term.data() + orientation_term.rows() * orientation_term.cols());

  // Add new orientation term to storage
  orientations_storage_->push_back(orientation_in);

  // Update cumulators and output for each channel of the orientation
  orientation_cumulator_ = orientation_cumulator_ - removed_orientation_ + orientation_term;
  Matrix4x<T> M = orientation_cumulator_ / orientations_storage_->size();

  // Select the eigenvector corresponding to the greatest eigenvalue of M
  Eigen::JacobiSVD<Matrix4x<T>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix4x<T> U = svd.matrixU();

  for (std::size_t i = position_in.size(); i < number_of_channels_; i++)
    data_out[i] = U(i - position_in.size(), 0);

  return true;
}

PLUGINLIB_EXPORT_CLASS(moving_average_poses_filter::MovingAveragePosesFilter<double>, filters::MultiChannelFilterBase<double>)
