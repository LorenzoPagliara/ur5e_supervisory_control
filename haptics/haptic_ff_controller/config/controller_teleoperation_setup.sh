#!/bin/bash

# Messaggio di inizio
echo "Start configuring controllers for teleoperation..."

# Esegui i comandi senza stampe intermedie
ros2 control list_controllers
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller
ros2 control unload_controller scaled_joint_trajectory_controller
ros2 control unload_controller forward_position_controller

ros2 control load_controller admittance_controller
ros2 control set_controller_state admittance_controller inactive
ros2 control set_controller_state admittance_controller active

ros2 control load_controller haptic_ff_controller
ros2 control set_controller_state haptic_ff_controller inactive
ros2 control set_controller_state haptic_ff_controller active

ros2 control list_controllers
# Messaggio di fine
echo "Controllers correctly configured for teleoperation."
