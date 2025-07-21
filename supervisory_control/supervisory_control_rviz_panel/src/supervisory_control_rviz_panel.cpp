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
 * Refer to the header file for a description of this module.
 * -------------------------------------------------------------------
 */

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "supervisory_control_rviz_panel/supervisory_control_rviz_panel.hpp"

namespace supervisory_control_rviz_panel
{
const std::vector<std::string> SupervisoryControlPanel::BUTTONS = { "next", "repeat", "manual" };

SupervisoryControlPanel::SupervisoryControlPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  icons_path_ = ament_index_cpp::get_package_share_directory("supervisory_control_rviz_panel") + "/icons/classes/";

  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  step_label_ = new QLabel("<b>Procedure step</b>: [not started]");
  message_label_ = new QLabel("<b>Message</b>: [no message]");

  next_button_ = new QPushButton("Next Step\t\t\t");
  next_button_->setIcon(QIcon(QString::fromStdString(icons_path_ + "next.png")));
  next_button_->setIconSize(QSize(24, 24));
  next_button_->setLayoutDirection(Qt::RightToLeft);

  repeat_button_ = new QPushButton("Repeat Step\t\t\t");
  repeat_button_->setIcon(QIcon(QString::fromStdString(icons_path_ + "repeat.png")));
  repeat_button_->setIconSize(QSize(24, 24));
  repeat_button_->setLayoutDirection(Qt::RightToLeft);

  manual_button_ = new QPushButton("Manual Control\t\t\t");
  manual_button_->setIcon(QIcon(QString::fromStdString(icons_path_ + "manual.png")));
  manual_button_->setIconSize(QSize(24, 24));
  manual_button_->setLayoutDirection(Qt::RightToLeft);
  is_manual_control_active_ = false;

  layout->addWidget(step_label_);
  layout->addWidget(message_label_);
  layout->addWidget(next_button_);
  layout->addWidget(repeat_button_);
  layout->addWidget(manual_button_);

  // Connect the button's click event to a slot
  QObject::connect(next_button_, &QPushButton::released, this, &SupervisoryControlPanel::next_button_activated_);
  QObject::connect(repeat_button_, &QPushButton::released, this, &SupervisoryControlPanel::repeat_button_activated_);
  QObject::connect(manual_button_, &QPushButton::released, this, &SupervisoryControlPanel::manual_button_activated_);
}

SupervisoryControlPanel::~SupervisoryControlPanel() = default;

void SupervisoryControlPanel::onInitialize()
{
  // Access the abstract ROS Node and in the process lock it for exclusive use until the method is done.
  node_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_->get_raw_node();
  buttons_publisher_ = node->create_publisher<std_msgs::msg::String>("/supervisory_control/validation", 10);
  state_subscription_ = node->create_subscription<supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState>(
      "/supervisory_control/rviz_state", 10, std::bind(&SupervisoryControlPanel::state_topic_callback_, this, std::placeholders::_1));
}

void SupervisoryControlPanel::state_topic_callback_(const supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState& msg)
{
  step_label_->setText(QString("<b>Procedure step</b>: %1").arg(QString::fromStdString(msg.state.data)));
  message_label_->setText(QString("<b>Message</b>: %1").arg(QString::fromStdString(msg.message.data)));

  std::vector<std::string> enabled_buttons;
  enabled_buttons.reserve(msg.enabled_buttons.size());

  for (const auto& button : msg.enabled_buttons)
  {
    enabled_buttons.push_back(button.data);
  }

  for (const auto& button : BUTTONS)
  {
    if (std::find(enabled_buttons.begin(), enabled_buttons.end(), button) != enabled_buttons.end())
    {
      if (button == "next")
      {
        next_button_->setEnabled(true);
      }
      else if (button == "repeat")
      {
        repeat_button_->setEnabled(true);
      }
      else if (button == "manual")
      {
        manual_button_->setEnabled(true);
      }
    }
    else
    {
      if (button == "next")
      {
        next_button_->setEnabled(false);
      }
      else if (button == "repeat")
      {
        repeat_button_->setEnabled(false);
      }
      else if (button == "manual")
      {
        manual_button_->setEnabled(false);
      }
    }
  }
}

void SupervisoryControlPanel::next_button_activated_()
{
  auto message = std_msgs::msg::String();
  message.data = "next";
  buttons_publisher_->publish(message);
}

void SupervisoryControlPanel::repeat_button_activated_()
{
  auto message = std_msgs::msg::String();
  message.data = "repeat";
  buttons_publisher_->publish(message);
}

void SupervisoryControlPanel::manual_button_activated_()
{
  auto message = std_msgs::msg::String();

  // Toggle between manual and automatic control
  // The message content is just a placeholder; the actual logic should be handled by the backend
  // or the supervisory control system.
  if (!is_manual_control_active_)
  {
    message.data = "manual";
    manual_button_->setText("Automatic Control\t\t\t");
    manual_button_->setIcon(QIcon(QString::fromStdString(icons_path_ + "automatic.png")));
    is_manual_control_active_ = true;

    next_button_->setEnabled(false);
    repeat_button_->setEnabled(false);
  }
  else
  {
    message.data = "automatic";
    manual_button_->setText("Manual Control\t\t\t");
    manual_button_->setIcon(QIcon(QString::fromStdString(icons_path_ + "manual.png")));
    is_manual_control_active_ = false;

    next_button_->setEnabled(true);
    repeat_button_->setEnabled(true);
  }

  buttons_publisher_->publish(message);
}

}  // namespace supervisory_control_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(supervisory_control_rviz_panel::SupervisoryControlPanel, rviz_common::Panel)
