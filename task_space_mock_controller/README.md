# task_space_mock_controller

This package provides a mock controller for a task space reference generator.
It is a chainable controller that exports the desired task space interfaces, allowing it to receive a task space reference (for example, from a task space reference generator) and publish it to the `task_space_mock_controller/actual` topic.

## Controller Configuration

Control parameters for the task space mock controller:

- [`task_space_reference_interfaces`](./src/task_space_mock_controller_parameters.yaml#L0002): a string array that specifies the task space command interfaces that the controller will export.
    Valid values are: "pose", "twist", "acceleration", "wrench", and "wrench_derivative".
    The exported interfaces will have the following default name structure: `task_space_mock_controller/<interface>`.
- If the default name structure is not desired, the user can specify the custom names using the [`reference_interfaces_names_override`](./src/task_space_mock_controller_parameters.yaml#L0008) parameter.
- [`task_space_reference_frame`](./src/task_space_mock_controller_parameters.yaml#L0039): the reference frame that will be used for the task space reference the controller will publish.
- [`publish_frequency`](./src/task_space_mock_controller_parameters.yaml#L0045): the frequency at which the controller will try to publish the task space reference.
    The default value is 5 Hz.

## Demo

To test this controller in simulation, please refer to the documentation of [`acg_resources_ur10_moveit_config`](../acg_resources_ur10_moveit_config/README.md#simulation-with-task-space-reference-generator).
