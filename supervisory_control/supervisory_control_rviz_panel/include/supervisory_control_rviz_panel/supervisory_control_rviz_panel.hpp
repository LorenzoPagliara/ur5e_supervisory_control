/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   supervisory_control_rviz_panel.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Jun 20, 2025
 *
 * Custom RViz panel for supervisory control.
 * -------------------------------------------------------------------
 */

#pragma once

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <supervisory_control_rviz_msg/msg/supervisory_controller_r_viz_state.hpp>

namespace supervisory_control_rviz_panel
{
class SupervisoryControlPanel : public rviz_common::Panel
{
  static const std::vector<std::string> BUTTONS;

  Q_OBJECT
public:
  /**
   * @brief Construct a SupervisoryControlPanel object.
   */
  explicit SupervisoryControlPanel(QWidget* parent = 0);

  /**
   * @brief Destroy a SupervisoryControlPanel object.
   */
  ~SupervisoryControlPanel() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  void onInitialize() override;

protected:
  /**
   * @brief Pointer to the ROS node abstraction interface.
   */
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_;

  /**
   * @brief Publisher for the buttons action.
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr buttons_publisher_;

  /**
   * @brief Subscription to the state updates.
   */
  rclcpp::Subscription<supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState>::SharedPtr state_subscription_;

  /**
   * @brief Callback for the state updates.
   * @param msg The message containing the step information.
   */
  void state_topic_callback_(const supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState& msg);

  /**
   * @brief Label to display the current step of the procedure.
   */
  QLabel* step_label_;

  /**
   * @brief Label to display a message for the user.
   */
  QLabel* message_label_;

  /**
   * @brief Button to proceed to the next automatic step.
   */
  QPushButton* next_button_;

  /**
   * @brief Button to repeat the current step.
   */
  QPushButton* repeat_button_;

  /**
   * @brief Button to switch to manual control.
   */
  QPushButton* manual_button_;

  /**
   * @brief Flag indicating whether manual control is active.
   */
  bool is_manual_control_active_ = false;

  /**
   * @brief Path to the icons used in the panel.
   */
  std::string icons_path_;

private Q_SLOTS:
  /**
   * @brief Function to be called when the next button is activated.
   */
  void next_button_activated_();

  /**
   * @brief Function to be called when the repeat button is activated.
   */
  void repeat_button_activated_();

  /**
   * @brief Function to be called when the manual button is activated.
   */
  void manual_button_activated_();
};

}  // namespace supervisory_control_rviz_panel
