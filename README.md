# robotic-arm

I'm building a **digital twin of a robotic cell that assembles timber wall panels**: an industrial arm picks studs from a stack and places them on an assembly table, following a panel design. It runs in **NVIDIA Isaac Sim**, it talks to the robot software through **ROS 2**, and every behaviour is checked by **automated scenario tests**.

> Work in progress.

## Why this project
Most robot simulations I see are demos: they look convincing, but nobody uses them to make engineering decisions. I want to build the other kind: a simulation that other engineers can run without me, that stays aligned with the hardware, and that answers concrete questions (Does the stud slip in the gripper? Is the cycle time realistic? What happens if a stud is missing from the stack?).

## What I'm building
- **The cell.** A UR10e arm, a stud stack, an assembly table with stops and a vacuum gripper, built in Isaac Sim from engineering geometry.
- **Panel designs as input.** A wall panel is described in a small JSON file (studs, top and bottom plates, openings). A Python planner turns it into a pick-and-place sequence.
- **One ROS 2 interface for sim and robot.** The robot software sends the same commands (`ros2_control` joint trajectories and gripper actions) whether it runs against Isaac Sim or a real arm.
- **Physics that I can defend.** Contacts, friction, joint drive stiffness and damping, and gripper behaviour tuned against reference values, with notes on where the simulation is good enough and where it is not.
- **Scenario tests.** YAML scenarios (nominal panel, missing stud, warped stud, heavy stud, gripper failure) run headless and report pass/fail, cycle time and placement error.
- **Two fidelity levels.** The same scenarios also run in **MuJoCo**, which is fast and runs on my Mac and on GitHub Actions. Isaac Sim, the high-fidelity version, runs on a cloud GPU. Comparing both shows what each level of fidelity can and cannot answer.
- **Synthetic data.** Isaac Sim Replicator generates labelled images of studs with domain randomisation (lighting, wood texture, pose) for a stud detector.

## Roadmap
1. **Cell and arm in MuJoCo.** UR10e (from MuJoCo Menagerie), table and studs; a scripted pick-and-place on my Mac.
2. **Panel planner.** Panel JSON → ordered list of placements, pure Python, unit-tested with pytest.
3. **ROS 2 interface.** A `ros2_control` hardware interface for the simulator, so the same trajectory controller drives sim and robot. Runs in Docker (ROS 2 Jazzy).
4. **Scenarios and CI.** YAML scenarios, metrics with no ROS imports, a GitHub Actions matrix with one job per scenario (MuJoCo backend).
5. **Isaac Sim cell.** The same cell in Isaac Sim on a cloud GPU: URDF → USD asset pipeline, physics tuning, ROS 2 bridge, same scenarios headless.
6. **Fidelity report.** Where MuJoCo and Isaac Sim disagree, why, and which one to trust for each question.
7. **Synthetic data.** Replicator pipeline with domain randomisation and versioned datasets.

## Tools
Python · ROS 2 Jazzy · ros2_control · NVIDIA Isaac Sim · MuJoCo · OpenUSD · Docker · pytest · GitHub Actions
