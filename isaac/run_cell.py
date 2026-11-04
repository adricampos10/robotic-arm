"""Import the ABB IRB 1200 into Isaac Sim and either check it or bridge it to ROS 2.

Written for Isaac Sim 6.2. Run it with Isaac Sim's Python (``./python.sh`` in
the Isaac Sim folder or container, or ``python`` with the isaacsim pip
package), after ``isaac/export_urdf.sh``:

    # No ROS needed: import, then drive the joints from Python and check tracking
    python isaac/run_cell.py --mode selftest --headless

    # ROS 2 bridge for hardware:=isaac (topic_based_ros2_control)
    python isaac/run_cell.py --mode ros [--headless]

Steps:
1. URDF -> USD with Isaac Sim's URDF importer. Fixed base, position drives on
   every joint, stiffness and damping from drive_gains.py (not hand-tuned).
2. A new stage with the robot referenced at /World/abb_irb1200 and a light.
3. --mode ros: an OmniGraph that publishes /isaac_joint_states and /clock and
   applies /isaac_joint_commands to the articulation.
   --mode selftest: a short trajectory sent straight to the articulation;
   reports gravity sag, tracking error, final error and the gains before and
   after I set them, and exits 0 if the final error is within tolerance.
In both modes the drive gains are set on the running articulation, so they
are drive_gains.py's values whatever the import left.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "build")

parser = argparse.ArgumentParser(description="ABB IRB 1200 in Isaac Sim")
parser.add_argument("--mode", choices=["selftest", "ros"], default="selftest")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--urdf", default=os.path.join(BUILD, "abb_irb1200.urdf"))
parser.add_argument("--packages", default=os.path.join(BUILD, "packages"),
                    help="Folder with the ROS packages the URDF's package:// paths point to.")
parser.add_argument("--usd-dir", default=os.path.join(BUILD, "usd"))
parser.add_argument("--physics-hz", type=float, default=120.0)
parser.add_argument("--frequency-hz", type=float, default=5.0, help="Drive natural frequency.")
parser.add_argument("--damping-ratio", type=float, default=1.0)
parser.add_argument("--max-static-error", type=float, default=0.002, help="Gravity sag, rad.")
parser.add_argument("--tolerance", type=float, default=0.01, help="Self-test final error, rad.")
parser.add_argument("--frames", type=int, default=0,
                    help="--mode ros: stop after this many frames (0 = run until closed).")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402

ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"]:
    ext_manager.set_extension_enabled_immediate(ext, True)
if args.mode == "ros":
    app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import UsdLux  # noqa: E402

sys.path.insert(0, HERE)
from drive_gains import compute_gains, report  # noqa: E402

ROBOT_PATH = "/World/abb_irb1200"
JOINTS = [f"joint_{i}" for i in range(1, 7)]
# Same waypoints as send_test_trajectory.py: (seconds from start, positions).
WAYPOINTS = [
    (2.0, [0.5, 0.3, -0.3, 0.5, 0.5, 0.5]),
    (4.0, [-0.5, -0.2, 0.3, -0.5, -0.5, -0.5]),
    (6.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


def import_robot(gains) -> str:
    config = URDFImporterConfig()
    config.urdf_path = os.path.abspath(args.urdf)
    config.usd_path = os.path.abspath(args.usd_dir)
    config.fix_base = True
    config.merge_fixed_joints = False  # keep tool0 and flange as frames
    config.joint_drive_type = "force"
    config.joint_target_type = "position"
    # Per-joint values, keyed by joint-name pattern.
    config.override_joint_stiffness = {f"^{g.joint}$": g.stiffness for g in gains}
    config.override_joint_damping = {f"^{g.joint}$": g.damping for g in gains}
    config.ros_package_paths = [
        {"name": name, "path": os.path.join(os.path.abspath(args.packages), name)}
        for name in sorted(os.listdir(args.packages))
    ]
    usd = URDFImporter(config).import_urdf()
    if not usd:
        raise RuntimeError(f"URDF import failed: {args.urdf}")
    print(f"[run_cell] Imported {args.urdf} -> {usd}")
    return usd


def build_stage(robot_usd: str) -> str:
    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage_utils.add_reference_to_stage(usd_path=robot_usd, path=ROBOT_PATH)
    stage = stage_utils.get_current_stage()
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
    simulation_app.update()
    roots = Articulation.fetch_articulation_root_api_prim_paths(ROBOT_PATH)
    if len(roots) != 1:
        raise RuntimeError(f"Expected one articulation root under {ROBOT_PATH}, found {roots}")
    stage.Export(os.path.join(BUILD, "abb_cell.usda"))
    print(f"[run_cell] Articulation root: {roots[0]}")
    return roots[0]


def build_ros_graph(articulation_root: str) -> None:
    """Joint states out, joint commands in, sim time on /clock."""
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2Bridge", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand",
                 "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand",
                 "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand",
                 "ArticulationController.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", articulation_root),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(articulation_root)]),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ],
        },
    )


def interpolate(t: float) -> np.ndarray:
    """Piecewise-linear position along WAYPOINTS, starting from zero at t=0."""
    previous_t, previous_q = 0.0, np.zeros(6)
    for waypoint_t, q in WAYPOINTS:
        if t <= waypoint_t:
            s = (t - previous_t) / (waypoint_t - previous_t)
            return previous_q + s * (np.array(q) - previous_q)
        previous_t, previous_q = waypoint_t, np.array(q)
    return previous_q


def set_gains(articulation_root: str, gains):
    """Check the joint names and make PhysX use my gains.

    The importer already writes them, but its later physics conversion step
    may change them. I report what the import left and then set them on the
    running articulation, so both modes use exactly drive_gains.py's values.
    Returns the articulation, the index of each joint in JOINTS order, and
    what the import left (stiffness, damping).
    """
    arm = Articulation(articulation_root)
    names = list(arm.dof_names)
    print(f"[run_cell] DOF names: {names}")
    if sorted(names) != JOINTS:
        raise RuntimeError(f"Expected joints {JOINTS}, got {names}")
    order = [names.index(j) for j in JOINTS]

    imported_k, imported_c = (np.asarray(a.numpy())[0][order] for a in arm.get_dof_gains())
    k = np.array([g.stiffness for g in gains])
    c = np.array([g.damping for g in gains])
    print("[run_cell] Imported / expected stiffness:", np.round(imported_k / k, 3).tolist())
    print("[run_cell] Imported / expected damping:  ", np.round(imported_c / c, 3).tolist())

    k_physx, c_physx = np.empty(6), np.empty(6)
    k_physx[order], c_physx[order] = k, c
    arm.set_dof_gains(k_physx[np.newaxis, :], c_physx[np.newaxis, :])
    return arm, order, (imported_k, imported_c)


def selftest(arm, order, imported, gains, dt: float) -> bool:
    stiffness, damping = (np.asarray(a.numpy())[0][order] for a in arm.get_dof_gains())
    expected_k = np.array([g.stiffness for g in gains])

    def positions() -> np.ndarray:
        return np.asarray(arm.get_dof_positions().numpy())[0][order]

    # Settle at zero first: shows the gravity sag on its own.
    for _ in range(int(1.0 / dt)):
        arm.set_dof_position_targets(np.zeros((1, 6))[:, order])
        simulation_app.update()
    sag = np.abs(positions())

    duration = WAYPOINTS[-1][0] + 1.0  # one extra second to settle
    tracking = np.zeros(6)
    for step in range(int(duration / dt)):
        target = interpolate(step * dt)
        command = np.empty(6)
        command[order] = target
        arm.set_dof_position_targets(command[np.newaxis, :])
        simulation_app.update()
        tracking = np.maximum(tracking, np.abs(positions() - target))
    final = np.abs(positions() - np.array(WAYPOINTS[-1][1]))

    ok = bool(final.max() <= args.tolerance)
    result = {
        "pass": ok,
        "physics_hz": args.physics_hz,
        "tolerance_rad": args.tolerance,
        "joints": JOINTS,
        "gravity_sag_rad": sag.round(5).tolist(),
        "max_tracking_error_rad": tracking.round(5).tolist(),
        "final_error_rad": final.round(5).tolist(),
        "stiffness_expected": expected_k.round(2).tolist(),
        "stiffness_physx": stiffness.round(2).tolist(),
        "damping_physx": damping.round(3).tolist(),
        "stiffness_after_import": imported[0].round(2).tolist(),
        "damping_after_import": imported[1].round(3).tolist(),
    }
    with open(os.path.join(BUILD, "selftest.json"), "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))
    return ok


def main() -> int:
    dt = 1.0 / args.physics_hz
    with open(args.urdf) as f:
        gains = compute_gains(f.read(), 2 * math.pi * args.frequency_hz,
                              args.damping_ratio, args.max_static_error)
    print(report(gains, dt))

    robot_usd = import_robot(gains)
    articulation_root = build_stage(robot_usd)
    if args.mode == "ros":
        build_ros_graph(articulation_root)
        simulation_app.update()

    SimulationManager.setup_simulation(dt=dt, device="cpu")
    app_utils.play()
    simulation_app.update()
    arm, order, imported = set_gains(articulation_root, gains)

    if args.mode == "selftest":
        ok = selftest(arm, order, imported, gains, dt)
        print("[run_cell] PASS" if ok else "[run_cell] FAIL")
        exit_code = 0 if ok else 1
    else:
        print("[run_cell] Bridge running: /isaac_joint_states, /isaac_joint_commands, /clock")
        frame = 0
        while simulation_app.is_running() and (args.frames == 0 or frame < args.frames):
            simulation_app.update()
            frame += 1
        exit_code = 0

    app_utils.stop()
    simulation_app.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
