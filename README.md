# robotic-arm

I'm building a **digital twin of a robotic cell that assembles timber wall panels**: an industrial arm picks studs from a stack and places them on an assembly table, following a panel design. It runs in **NVIDIA Isaac Sim**, it talks to the robot software through **ROS 2**, and every behaviour is checked by **automated tests**.

> Work in progress.

## What I'm building
- **An ABB arm driven through ROS 2.** I start with the ABB IRB 1200 from [`abb_ros2`](https://github.com/PickNikRobotics/abb_ros2), the package that also has the EGM driver for the real controller. A larger IRB 4600, sized for timber studs, comes next.
- **One ROS 2 interface for sim and robot.** Controllers, MoveIt and my code are the same whether the arm is mock hardware, Isaac Sim or a real ABB controller. Only the `ros2_control` hardware plugin changes, chosen with one launch argument.
- **The cell in Isaac Sim.** The arm imported from URDF to USD, joint drives set up, and a ROS 2 bridge. Then a stud stack, an assembly table and a vacuum gripper.
- **Physics that I can defend.** Joint drive stiffness and damping derived from the robot's masses and inertias rather than tuned by eye, then contacts and friction, with notes on where the simulation is good enough and where it is not.
- **Tests that run anywhere.** Everything except Isaac Sim runs in Docker on a laptop and in CI, so GPU time is only spent checking work that already passes on mock hardware.

## Quick start (mock hardware, no GPU)
```bash
docker build -f docker/Dockerfile -t robotic-arm:jazzy .

# Bring up the arm on mock hardware
docker run --rm -it robotic-arm:jazzy ros2 launch abb_cell_bringup bringup.launch.py hardware:=mock

# Run the ROS tests (xacro for each hardware target + a trajectory on mock hardware)
docker run --rm robotic-arm:jazzy bash -c "cd /ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose"

# Run the tests that need neither ROS nor Isaac Sim (drive gains)
pip install -r requirements-dev.txt && pytest
```

| `hardware:=` | `ros2_control` plugin | Where |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | Laptop, CI |
| `isaac` | `topic_based_ros2_control/TopicBasedSystem` (`/isaac_joint_states`, `/isaac_joint_commands`) | GPU machine running Isaac Sim |
| `real` | `abb_hardware_interface/ABBSystemHardware` (EGM) | ABB controller (driver not built in the image yet) |

`send_test_trajectory.py` sends a short trajectory and checks that every joint gets there. I use the same check on each target.

## Layout
- `ros2_ws/src/abb_cell_description`: URDF/xacro with the hardware selector.
- `ros2_ws/src/abb_cell_bringup`: launch file, controller configuration, trajectory check and launch test.
- `docker/`: ROS 2 Jazzy image with ros2_control, MoveIt and the upstream ABB packages.
- `isaac/export_urdf.sh`: a plain URDF and its mesh packages, exported from the ROS 2 image into `isaac/build/` for Isaac Sim's importer.
- `isaac/drive_gains.py`: joint drive stiffness and damping from the URDF (pure Python, tested with pytest).
- `isaac/run_cell.py`: the Isaac Sim script, written for Isaac Sim 6.2: URDF → USD import, drives, then a self-test or the ROS 2 bridge.

## Joint drives: stiffness and damping from the physics
Isaac Sim (PhysX) moves each joint towards its target with an implicit PD law:

```
τ = k (q_target − q) + c (q̇_target − q̇)
```

Guessing `k` and `c` until the arm "looks right" hides problems: too soft and the arm sags under its own weight, too stiff for the time step and the motion is distorted. So I derive both per joint from the masses and inertias in the URDF.

**1. What each joint has to move.** For each joint I add up every link downstream of it, with the other joints at zero:
- the inertia about the joint axis, `I = Σ (aᵀ Rᵢ I_com,ᵢ Rᵢᵀ a + mᵢ dᵢ²)`, where `dᵢ` is the distance from link `i`'s centre of mass to the axis (parallel axis theorem);
- the worst gravity torque over the joint's own rotation, `τg = |Σ mᵢ rᵢ⊥| · |g⊥|`, where `rᵢ⊥` is the centre of mass offset perpendicular to the axis and `g⊥` the part of gravity perpendicular to it. A vertical axis, like joint 1, carries no gravity torque at all.

**2. Two requirements for the stiffness. I take the larger:**
- **response:** the joint should behave like a second-order system with natural frequency `ωn` (5 Hz by default), so `k = I ωn²`;
- **holding against gravity:** with a pure PD drive and no gravity compensation, the joint sags until `k · error = τg`. Keeping that sag under `e_max` (2 mrad by default) needs `k ≥ τg / e_max`.

**3. Damping for no overshoot.** `c = 2 ζ √(k I)` with `ζ = 1`, critical damping: the joint reaches its target as fast as it can without overshooting.

Result for the IRB 1200 (`python isaac/drive_gains.py isaac/build/abb_irb1200.urdf`):

| joint | I (kg m²) | τg (N m) | k (N m/rad) | c (N m s/rad) | limited by |
|---|---|---|---|---|---|
| joint_1 | 0.919 | 0.0 | 907 | 57.8 | response |
| joint_2 | 4.889 | 97.5 | 48 746 | 976 | gravity |
| joint_3 | 0.766 | 21.5 | 10 748 | 181 | gravity |
| joint_4 | 0.0073 | 0.14 | 71 | 1.44 | gravity |
| joint_5 | 0.0029 | 0.17 | 85 | 1.00 | gravity |
| joint_6 | 0.0010 | 0.0 | 1.0 | 0.06 | response |

What this shows:
- **Gravity sets most of the stiffnesses.** Joint 2 holds almost 100 N m, so it needs about 10 times the stiffness the 5 Hz response asks for (30 times on joint 5). The joints end up much stiffer than intended, up to ~27 Hz on joint 5.
- **The physics time step matters.** At Isaac Sim's default 60 Hz step, `ωn·dt` reaches almost 3. PhysX drives are implicit, so they stay stable, but the step is too coarse for that bandwidth and the response is distorted. That's why `run_cell.py` runs physics at 120 Hz.
- **The alternative is gravity compensation.** A real robot controller adds a feed-forward torque for gravity. Doing the same in simulation, or disabling gravity on the arm links, would let the drives be much softer. That's a fidelity choice I want to make on purpose, not by accident.
- **The values come from one reference pose.** With the other joints at zero the arm is folded, so joint 1's inertia is at its smallest; reaching out it grows and the response slows.

These are predicted values. The self-test in Isaac Sim measures the sag and the tracking error to check them.

## Running in Isaac Sim
Everything is prepared and tested on a laptop. A GPU session only runs the script and records the results, so it should take minutes. Isaac Sim 6.2.0 needs Linux, Python 3.12 and an NVIDIA GPU with RT cores.

### Session 1: import and self-test, no ROS (free GPU: Colab or Kaggle)
On the laptop, run `isaac/export_urdf.sh` and zip `isaac/` (including `isaac/build/`). In a notebook with a GPU (T4 or better):

```bash
pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com
export OMNI_KIT_ACCEPT_EULA=YES
python isaac/run_cell.py --mode selftest --headless
```

Check and keep:
- [ ] The import succeeds and prints `Articulation root: ...`. If the `.dae` visuals fail, re-export with `isaac/export_urdf.sh --stl-visuals`.
- [ ] `DOF names` are `joint_1` … `joint_6` (`topic_based_ros2_control` matches joints by name).
- [ ] The `Imported / expected` stiffness and damping ratios. 1.0 means the importer kept my gains; anything else means its physics conversion step changed them. The script sets the right values either way, but it's worth knowing.
- [ ] `gravity_sag_rad` at most ~0.002 per joint, as predicted above.
- [ ] `max_tracking_error_rad` during the motion, and `final_error_rad` ≤ 0.01: `PASS`.
- [ ] Download `isaac/build/selftest.json`, `isaac/build/usd/` and `isaac/build/abb_cell.usda`.

Colab and Kaggle are not supported Isaac Sim platforms. If the install or the first start fails there, session 1 moves to the session 2 machine.

### Session 2: the ROS 2 bridge (cheap Spot GPU instance with Docker)
Ubuntu 22.04/24.04, NVIDIA driver, Docker, an RTX-class GPU (A10G or L40S; not A100/H100). Stop the instance as soon as the checks pass.

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

`--ipc host` lets the containers share memory with Isaac Sim, which Fast DDS uses for transport between processes on the same machine.

Check and keep:
- [ ] `ros2 topic hz /isaac_joint_states` is close to the physics rate, and `/clock` is published.
- [ ] `ros2 control list_controllers`: both controllers `active`.
- [ ] `send_test_trajectory.py` prints `PASS`. Keep the `Largest final error` line.
- [ ] Optional: `moveit:=true`, and plan to a pose with the MoveIt Python API.

### If something goes wrong
| Symptom | First thing to check |
|---|---|
| Joints droop and never reach the target | Stiffness lost at import (see the ratios); measured sag vs `τg / k` |
| Joints oscillate or jitter | Damping ratio (should be ~1), physics step (`ωn·dt`), solver iterations |
| Arm explodes or flies off | Base not fixed, self-collision between adjacent links, wrong inertia units |
| No `/isaac_joint_states` in the ROS container | Same `ROS_DOMAIN_ID` and discovery range on both sides, `--network host --ipc host` |
| Controller goal fails with a tolerance error | Tracking lag vs `goal_time` and `goal` in `controllers.yaml`; sim time on both sides |

## Roadmap
1. **ABB arm in ROS 2.** ros2_control, controllers and MoveIt with a hardware selector. The trajectory check passes on mock hardware in Docker; planning through MoveIt is next.
2. **Isaac Sim bridge.** Import the IRB 1200 (URDF → USD), joint drives with gains derived from the URDF, and the OmniGraph ROS 2 bridge, set up by a headless Python script. The script and the gain tests are written; the run on a GPU is next.
3. **CI.** GitHub Actions builds the image and runs the tests on every push.
4. **The cell.** IRB 4600, table, stud stack and vacuum gripper; a panel JSON turned into a pick-and-place sequence.
5. **Physics and scenarios.** Contact and friction tuning, gravity compensation; YAML scenarios (missing, warped or heavy stud, gripper failure) with pass/fail metrics.
6. **Synthetic data.** Replicator pipeline with domain randomisation.

## Tools
Python · ROS 2 Jazzy · ros2_control · MoveIt 2 · NVIDIA Isaac Sim · OpenUSD · Docker · pytest
