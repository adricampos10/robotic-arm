"""The xacro expands for each hardware target and picks the right plugin."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

XACRO = os.path.join(
    get_package_share_directory("abb_cell_description"), "urdf", "abb_cell.urdf.xacro"
)
JOINTS = [f"joint_{i}" for i in range(1, 7)]


def expand(hardware: str) -> ET.Element:
    urdf = subprocess.run(
        ["xacro", XACRO, f"hardware:={hardware}"],
        check=True, capture_output=True, text=True,
    ).stdout
    return ET.fromstring(urdf)


@pytest.mark.parametrize(
    "hardware, plugin",
    [
        ("mock", "mock_components/GenericSystem"),
        ("isaac", "topic_based_ros2_control/TopicBasedSystem"),
        ("real", "abb_hardware_interface/ABBSystemHardware"),
    ],
)
def test_plugin_per_hardware(hardware, plugin):
    robot = expand(hardware)
    assert robot.find("ros2_control/hardware/plugin").text == plugin


@pytest.mark.parametrize("hardware", ["mock", "isaac", "real"])
def test_same_joints_everywhere(hardware):
    robot = expand(hardware)
    names = [j.get("name") for j in robot.findall("ros2_control/joint")]
    assert names == JOINTS
    for joint in robot.findall("ros2_control/joint"):
        commands = [c.get("name") for c in joint.findall("command_interface")]
        assert commands[0] == "position"


def test_isaac_topics():
    robot = expand("isaac")
    params = {p.get("name"): p.text for p in robot.findall("ros2_control/hardware/param")}
    assert params["joint_commands_topic"] == "/isaac_joint_commands"
    assert params["joint_states_topic"] == "/isaac_joint_states"


def test_unknown_hardware_fails():
    with pytest.raises(subprocess.CalledProcessError):
        expand("gazebo")
