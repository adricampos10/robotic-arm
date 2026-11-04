"""Bring up the cell on mock hardware and run the trajectory check against it."""

import os
import unittest

import launch_testing
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("abb_cell_bringup"), "launch", "bringup.launch.py"
        )),
        launch_arguments={"hardware": "mock"}.items(),
    )
    check = Node(
        package="abb_cell_bringup",
        executable="send_test_trajectory.py",
        output="screen",
    )
    return LaunchDescription([bringup, check, launch_testing.actions.ReadyToTest()]), {
        "check": check
    }


class TestTrajectory(unittest.TestCase):
    def test_check_finishes(self, proc_info, check):
        proc_info.assertWaitForShutdown(process=check, timeout=90)


@launch_testing.post_shutdown_test()
class TestExitCode(unittest.TestCase):
    def test_check_passed(self, proc_info, check):
        launch_testing.asserts.assertExitCodes(proc_info, process=check)
