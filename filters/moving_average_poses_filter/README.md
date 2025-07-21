# moving_average_poses_filter

In this package, the `MovingAveragePosesFilter` class is provided, extending the `filters::MultiChannelFilterBase` base class of the filters ROS package.

This package is a modified version of the multi_channel_moving_average_filter for poses, expressed as standard vectors. For general explanations about the theoretical formulation, you should refer to the [moving_average_filter](https://bitbucket.org/eferre89/unisa_acg_ros2/src/2c385eb03b2ab5d1fd4721a75a791cbf315cefcc/filters/moving_average_filter/README.md?at=feature%2Fmoving_average_filter_utility) package and to the [document](https://bitbucket.org/eferre89/unisa_acg_ros2/src/584bc4ede331b87af23bec3befb2b6350ca38e3d/filters/moving_average_poses_filter/doc/NASA%20-%202007%20-%20Quaternion%20Averaging.pdf?at=feature%2Fmoving_average_poses_filter) about quaternion averaging.

## How to configure

In the case of the `MovingAveragePosesFilter` class, the filter-specific parameters needed are:

- `number_of_observations`: the size `n` of the moving window;

Examples of configuration files are present in the `test` folder.

## How to build

For importing and using this package, you need to build it first:

```sh
colcon build --packages-select moving_average_poses_filter
```

Do not forget to source your workspace after built:
```sh
cd ros2_ws
source install/setup.bash
```

## How to test
The tests in this package cover the configuration procedure and the expected results from known inputs with different sample sizes. You may refer to [test_moving_average_poses_filter.cpp](https://bitbucket.org/eferre89/unisa_acg_ros2/src/deec2dab0456ab938d34008f8a94a1831b84e437/filters/moving_average_poses_filter/test/test_moving_average_poses_filter.cpp?at=feature%2Fmoving_average_poses_filter) for implementation details.

In order to perform a test, use the colcon tool. For instance:
```sh
colcon test --packages-select moving_average_poses_filter
```

To see the results of the tests, you can use the following command:
```sh
colcon test-result --test-result-base build/moving_average_poses_filter --verbose
```
