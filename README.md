# robotic-arm

I'm building a **digital twin of a robotic cell that assembles timber wall panels**: an industrial arm picks studs from a stack and places them on an assembly table, following a panel design. It runs in **NVIDIA Isaac Sim**, it talks to the robot software through **ROS 2**, and every behaviour is checked by **automated tests**.

> Work in progress.

## What I'm building
- **An ABB arm driven through ROS 2.** I start with the ABB IRB 1200 from [`abb_ros2`](https://github.com/PickNikRobotics/abb_ros2), the package that also has the EGM driver for the real controller. A larger IRB 4600, sized for timber studs, comes next.
- **One ROS 2 interface for sim and robot.** Controllers, MoveIt and my code are the same whether the arm is mock hardware, Isaac Sim or a real ABB controller. Only the `ros2_control` hardware plugin changes, chosen with one launch argument.
- **The cell in Isaac Sim.** The arm imported from URDF to USD, joint drives tuned, and a ROS 2 bridge. Then a stud stack, an assembly table and a vacuum gripper.
- **Physics that I can defend.** Contacts, friction, joint drive stiffness and damping, tuned against reference values, with notes on where the simulation is good enough and where it is not.
- **Tests that run anywhere.** Everything except Isaac Sim runs in Docker on a laptop and in CI, so GPU time is only spent checking work that already passes on mock hardware.

## Quick start (mock hardware, no GPU)
```bash
docker build -f docker/Dockerfile -t robotic-arm:jazzy .

# Bring up the arm on mock hardware
docker run --rm -it robotic-arm:jazzy ros2 launch abb_cell_bringup bringup.launch.py hardware:=mock

# Run the tests (xacro for each hardware target + a trajectory on mock hardware)
docker run --rm robotic-arm:jazzy bash -c "cd /ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose"
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

## Roadmap
1. **ABB arm in ROS 2.** ros2_control, controllers and MoveIt with a hardware selector. The trajectory check passes on mock hardware in Docker; planning through MoveIt is next.
2. **Isaac Sim bridge.** Import the IRB 1200 (URDF → USD), set up joint drives and the OmniGraph ROS 2 bridge, and run the same trajectory check with `hardware:=isaac`. A headless Python script sets this up.
3. **CI.** GitHub Actions builds the image and runs the tests on every push.
4. **The cell.** IRB 4600, table, stud stack and vacuum gripper; a panel JSON turned into a pick-and-place sequence.
5. **Physics and scenarios.** Contact, friction and drive tuning; YAML scenarios (missing, warped or heavy stud, gripper failure) with pass/fail metrics.
6. **Synthetic data.** Replicator pipeline with domain randomisation.

## Tools
Python · ROS 2 Jazzy · ros2_control · MoveIt 2 · NVIDIA Isaac Sim · OpenUSD · Docker · pytest
