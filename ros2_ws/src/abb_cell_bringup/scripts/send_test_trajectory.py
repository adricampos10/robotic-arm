#!/usr/bin/env python3
"""Send a short joint trajectory to the arm and check that it gets there.

It works the same against mock hardware, Isaac Sim and the real robot, so I
use it as the first check on any new setup:

    ros2 run abb_cell_bringup send_test_trajectory.py
    ros2 run abb_cell_bringup send_test_trajectory.py --ros-args -p use_sim_time:=true

Exit code 0 if the controller reports success and every joint ends within
the tolerance of the last waypoint, 1 otherwise.
"""

import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 7)]

# (seconds from start, joint positions in rad). Small moves well inside the
# IRB 1200 limits, ending back at zero.
WAYPOINTS = [
    (2.0, [0.5, 0.3, -0.3, 0.5, 0.5, 0.5]),
    (4.0, [-0.5, -0.2, 0.3, -0.5, -0.5, -0.5]),
    (6.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


class TrajectoryCheck(Node):
    def __init__(self):
        super().__init__("send_test_trajectory")
        self.declare_parameter("controller", "joint_trajectory_controller")
        self.declare_parameter("tolerance", 0.02)
        self.declare_parameter("startup_timeout", 60.0)
        self.controller = self.get_parameter("controller").value
        self.client = ActionClient(
            self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory"
        )
        self.list_controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

    def on_joint_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_until_active(self, timeout: float) -> bool:
        """The action server exists as soon as the controller is configured,
        but it rejects goals until the controller is active."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.list_controllers.wait_for_service(timeout_sec=1.0):
                future = self.list_controllers.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                states = {c.name: c.state for c in (future.result() or
                          ListControllers.Response()).controller}
                if states.get(self.controller) == "active" and len(self.positions) >= 6:
                    return True
            rclpy.spin_once(self, timeout_sec=0.5)
        return False

    def run(self) -> bool:
        timeout = self.get_parameter("startup_timeout").value
        if not self.wait_until_active(timeout):
            self.get_logger().error(
                f"{self.controller} not active, or no joint states, after {timeout} s"
            )
            return False
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("No trajectory action server")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for t, q in WAYPOINTS:
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=q,
                time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9)),
            ))

        self.get_logger().info(f"Sending {len(WAYPOINTS)} waypoints")
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Controller error {result.error_code}: {result.error_string}"
            )
            return False

        # Let a few joint state messages arrive after the motion ends.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        target = dict(zip(JOINTS, WAYPOINTS[-1][1]))
        tolerance = self.get_parameter("tolerance").value
        errors = {j: abs(self.positions.get(j, float("inf")) - target[j]) for j in JOINTS}
        worst = max(errors, key=errors.get)
        self.get_logger().info(f"Largest final error: {worst} {errors[worst]:.4f} rad")
        return errors[worst] <= tolerance


def main():
    rclpy.init()
    node = TrajectoryCheck()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
