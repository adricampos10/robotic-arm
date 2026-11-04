# Isaac Sim: the IRB 1200 with tuned drives and a ROS 2 bridge

Everything here is prepared and tested on a laptop. A GPU session only runs
it and records the results, so it should take minutes. Written for
**Isaac Sim 6.2.0** (Python 3.12, Linux).

| File | What it does | Runs on |
|---|---|---|
| `export_urdf.sh` | Plain URDF + mesh packages from the ROS 2 image, into `build/` | Laptop (Docker) |
| `drive_gains.py` | Stiffness and damping per joint from the URDF's masses and inertias | Anywhere (pytest) |
| `run_cell.py` | URDF → USD import, drives, then `selftest` or `ros` mode | Isaac Sim (GPU) |

## How the drive gains are chosen
PhysX drives each joint with an implicit PD law, `τ = k (q* − q) + c (q̇* − q̇)`.
For each joint I compute the inertia it moves (`I`) and the worst gravity torque
it holds (`τg`), then take the larger of two stiffnesses:

- response: `k = I ωn²` (5 Hz by default)
- holding against gravity with no compensation: `k ≥ τg / e_max` (2 mrad by default)

and set `c = 2 ζ √(k I)` with `ζ = 1` (critically damped, no overshoot).

```bash
python isaac/drive_gains.py isaac/build/abb_irb1200.urdf
```

| joint | I (kg m²) | τg (N m) | k (N m/rad) | c (N m s/rad) | limited by |
|---|---|---|---|---|---|
| joint_1 | 0.919 | 0.0 | 907 | 57.8 | response |
| joint_2 | 4.889 | 97.5 | 48 746 | 976 | gravity |
| joint_3 | 0.766 | 21.5 | 10 748 | 181 | gravity |
| joint_4 | 0.0073 | 0.14 | 71 | 1.44 | gravity |
| joint_5 | 0.0029 | 0.17 | 85 | 1.00 | gravity |
| joint_6 | 0.0010 | 0.0 | 1.0 | 0.06 | response |

Gravity sets most of the stiffnesses, so the joints end up much stiffer than
the 5 Hz response asks for (up to ~27 Hz on joint 5). At Isaac's default 60 Hz
physics step that gives `ωn·dt` near 3. PhysX drives are implicit, so this
stays stable, but it is a coarse step for that bandwidth, which is why
`run_cell.py` runs physics at 120 Hz. The alternative is gravity compensation
(a feed-forward torque, or disabling gravity on the arm links), which would
let the joints be much softer. Inertia and gravity are computed with the other
joints at zero: joint 1's inertia grows when the arm reaches out, so its
response is slower there.

## Session 1: import and self-test, no ROS (free GPU: Colab or Kaggle)
On the laptop, export the URDF and zip `isaac/` (including `isaac/build/`).
In a notebook with a GPU (T4 or better; Isaac Sim needs RT cores):

```bash
pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com
export OMNI_KIT_ACCEPT_EULA=YES
python isaac/run_cell.py --mode selftest --headless
```

Check and keep:
- [ ] The import succeeds and prints `Articulation root: ...`. If the `.dae` visuals fail, re-export with `isaac/export_urdf.sh --stl-visuals`.
- [ ] `DOF names` are `joint_1` … `joint_6` (`topic_based_ros2_control` matches by name).
- [ ] `Imported / expected stiffness` and `damping` ratios. 1.0 means the importer kept my gains. Anything else means its physics conversion changed them, which is worth knowing even though the script then sets the right values.
- [ ] `gravity_sag_rad` at most ~0.002 per joint, as `drive_gains.py` predicts.
- [ ] `max_tracking_error_rad` during the motion and `final_error_rad` ≤ 0.01: `PASS`.
- [ ] Download `isaac/build/selftest.json`, `isaac/build/usd/` and `isaac/build/abb_cell.usda`.

Colab and Kaggle are not supported Isaac Sim platforms. If the install or the
first start fails there, session 1 moves to the session 2 machine.

## Session 2: the ROS 2 bridge (cheap Spot GPU instance with Docker)
Ubuntu 22.04/24.04, NVIDIA driver, Docker, an RTX-class GPU (A10G or L40S;
not A100/H100). Stop the instance as soon as the checks pass.

```bash
git clone https://github.com/adricampos10/robotic-arm.git && cd robotic-arm
docker build -f docker/Dockerfile -t robotic-arm:jazzy .
isaac/export_urdf.sh

# Terminal 1: Isaac Sim with the bridge (isaacsim pip package installed as in session 1)
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python isaac/run_cell.py --mode ros --headless

# Terminal 2: the same bringup as on the laptop, with hardware:=isaac
docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=0 robotic-arm:jazzy \
  ros2 launch abb_cell_bringup bringup.launch.py hardware:=isaac

# Terminal 3: the same trajectory check as on the laptop, on sim time
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=0 robotic-arm:jazzy \
  ros2 run abb_cell_bringup send_test_trajectory.py --ros-args -p use_sim_time:=true
```

`--ipc host` lets the containers share memory with Isaac Sim, which Fast DDS
uses for transport between processes on the same machine.

Check and keep:
- [ ] `ros2 topic hz /isaac_joint_states` is close to the physics rate, and `/clock` is published.
- [ ] `ros2 control list_controllers`: both controllers `active`.
- [ ] `send_test_trajectory.py` prints `PASS`. Keep the `Largest final error` line.
- [ ] Optional: `moveit:=true` and plan to a pose with the MoveIt Python API.

## If something goes wrong
| Symptom | First thing to check |
|---|---|
| Joints droop and never reach the target | Stiffness lost at import (see the ratios); gravity sag vs `drive_gains.py` |
| Joints oscillate or jitter | Damping ratio (should be ~1), physics step (`ωn·dt`), solver iterations |
| Arm explodes or flies off | Base not fixed, self-collision between adjacent links, wrong inertia units |
| No `/isaac_joint_states` in the ROS container | Same `ROS_DOMAIN_ID` and discovery range on both sides, `--network host --ipc host` |
| Controller goal fails with tolerance error | Tracking lag vs `goal_time` and `goal` in `controllers.yaml`; check sim time on both sides |
