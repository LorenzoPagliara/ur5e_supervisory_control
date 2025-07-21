from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing import post_shutdown_test
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc

import pytest
from unittest import TestCase


@pytest.mark.launch_test
def generate_test_description():
    test_moving_average_filter_node = Node(
        package="moving_average_poses_filter",
        executable="moving_average_poses_filter_test",
        name="test_moving_average_filter_node",
        output="screen",
    )

    return LaunchDescription(
        [
            test_moving_average_filter_node,
            KeepAliveProc(),
            ReadyToTest(),
        ]
    ), {
        "test_moving_average_filter_node": test_moving_average_filter_node,
    }


class TestTerminatingProcessStops(TestCase):
    def test_gtest_run_complete(self, proc_info, test_moving_average_filter_node):
        proc_info.assertWaitForShutdown(
            process=test_moving_average_filter_node, timeout=4000.0
        )


@post_shutdown_test()
class TestOutcome(TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
