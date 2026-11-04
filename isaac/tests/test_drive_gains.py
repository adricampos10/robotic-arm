"""Drive gains on small arms whose answers I can work out by hand."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from drive_gains import compute_gains, rpy_to_matrix  # noqa: E402

G = 9.81


def pendulum(axis: str, mass=2.0, length=0.5, izz=0.01, joint_rpy="0 0 0") -> str:
    """One link, COM `length` along x from a revolute joint about `axis`."""
    return f"""
    <robot name="pendulum">
      <link name="base"/>
      <link name="arm">
        <inertial>
          <origin xyz="{length} 0 0"/>
          <mass value="{mass}"/>
          <inertia ixx="{izz}" iyy="{izz}" izz="{izz}" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
      <joint name="j" type="revolute">
        <origin xyz="0 0 1" rpy="{joint_rpy}"/>
        <parent link="base"/><child link="arm"/>
        <axis xyz="{axis}"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>"""


def test_horizontal_axis_inertia_and_gravity():
    # Axis y, COM 0.5 m along x: I = Icom + m L^2, worst gravity torque m g L.
    (g,) = compute_gains(pendulum("0 1 0"), natural_frequency=10.0, max_static_error=0.01)
    assert g.inertia == pytest.approx(0.01 + 2.0 * 0.25)
    assert g.gravity_torque == pytest.approx(2.0 * G * 0.5)


def test_vertical_axis_has_no_gravity_torque():
    (g,) = compute_gains(pendulum("0 0 1"), natural_frequency=10.0)
    assert g.gravity_torque == pytest.approx(0.0, abs=1e-9)
    assert g.limited_by == "response"
    assert g.stiffness == pytest.approx(g.inertia * 100.0)


def test_gravity_sets_stiffness_when_it_dominates():
    (g,) = compute_gains(pendulum("0 1 0"), natural_frequency=1.0, max_static_error=0.01)
    assert g.limited_by == "gravity"
    assert g.stiffness == pytest.approx(2.0 * G * 0.5 / 0.01)
    assert g.static_error == pytest.approx(0.01)


def test_critical_damping():
    (g,) = compute_gains(pendulum("0 0 1"), natural_frequency=10.0, damping_ratio=1.0)
    assert g.damping == pytest.approx(2 * math.sqrt(g.stiffness * g.inertia))
    assert g.natural_frequency == pytest.approx(10.0)


def test_joint_origin_rotation_turns_the_axis():
    # Rotating the joint frame 90 deg about x turns axis z into world -y: horizontal.
    (g,) = compute_gains(pendulum("0 0 1", joint_rpy=f"{math.pi / 2} 0 0"))
    assert g.gravity_torque == pytest.approx(2.0 * G * 0.5)


def test_rpy_convention():
    # yaw 90 deg maps x to y.
    assert rpy_to_matrix(0, 0, math.pi / 2) @ [1, 0, 0] == pytest.approx([0, 1, 0])


IRB1200 = os.path.join(os.path.dirname(__file__), "..", "build", "abb_irb1200.urdf")


@pytest.mark.skipif(not os.path.exists(IRB1200), reason="run isaac/export_urdf.sh first")
def test_irb1200_sanity():
    with open(IRB1200) as f:
        gains = {g.joint: g for g in compute_gains(f.read())}
    assert sorted(gains) == [f"joint_{i}" for i in range(1, 7)]
    # Joint 1 is vertical: no gravity torque. Joint 2 carries the most.
    assert gains["joint_1"].gravity_torque == pytest.approx(0.0, abs=1e-6)
    assert max(gains.values(), key=lambda g: g.gravity_torque).joint == "joint_2"
    # Inertia drops along the chain from the shoulder to the wrist.
    assert gains["joint_2"].inertia > gains["joint_4"].inertia > gains["joint_6"].inertia
