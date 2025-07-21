/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_poses_filter.h
 * Author:  Michele Marsico
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * This module implements the filters::MultiChannelFilterBase
 * interface and consists of a templated filter for poses.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <filters/filter_base.hpp>
#include <filters/realtime_circular_buffer.hpp>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

template <typename T>
using Matrix4x = typename Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

template <typename T>
using Vector4x = typename Eigen::Matrix<T, 4, 1>;

namespace moving_average_poses_filter
{
template <typename T>
class MovingAveragePosesFilter : public filters::MultiChannelFilterBase<T>
{
public:
  /**
   * @brief Construct a MovingAveragePosesFilter object.
   */
  MovingAveragePosesFilter();

  /**
   * @brief Destroy a MovingAveragePosesFilter object.
   */
  ~MovingAveragePosesFilter();

  /**
   * @brief Retrieve the configuration from the parameter server and configure the filter.
   *
   * @return true if configuration succeeds, false otherwise
   */
  bool configure();

  /**
   * @brief Update the filter and return the average.
   *
   * @param[in] data_in new observation
   * @param[out] data_out average value of observations within the moving window
   * @return true if update succeeds, false otherwise
   */
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out);

private:
  /**
   * @brief Positions within the current moving window.
   */
  std::unique_ptr<filters::RealtimeCircularBuffer<std::vector<T>>> positions_storage_;

  /**
   * @brief Orientation terms within the current moving window.
   */
  std::unique_ptr<filters::RealtimeCircularBuffer<std::vector<T>>> orientations_storage_;

  /**
   * @brief The maximum number of elements that the moving window can store.
   */
  uint32_t number_of_observations_;

  /**
   * @brief Size of each element. Retrieved from superclass after the configure() method.
   */
  using filters::MultiChannelFilterBase<T>::number_of_channels_;

  /**
   * @brief The number of channels used for the position.
   */
  const uint32_t number_of_position_channels_;

  /**
   * @brief The number of channels used for the orientation.
   */
  const uint32_t number_of_orientation_channels_;

  /**
   * @brief Flag indicating if the filter was previously configured.
   */
  using filters::MultiChannelFilterBase<T>::configured_;

  /**
   * @brief Zero position.
   */
  std::vector<T> position_zeros_;

  /**
   * @brief Sum of positions within the current moving window.
   */
  std::vector<T> position_cumulator_;

  /**
   * @brief The last removed position from the moving window.
   */
  std::vector<T> removed_position_;

  /**
   * @brief Zero orientation term.
   */
  Matrix4x<T> orientation_zeros_;

  /**
   * @brief Sum of orientation terms within the current moving window.
   */
  Matrix4x<T> orientation_cumulator_;

  /**
   * @brief The last removed orientation term from the moving window.
   */
  Matrix4x<T> removed_orientation_;

  /**
   * @brief Logger instance for the MultiChannelMovingAverageFilter class.
   */
  rclcpp::Logger logger_{ rclcpp::get_logger("MovingAveragePosesFilter") };
};

}  // namespace moving_average_poses_filter
