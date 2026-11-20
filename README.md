# Cooperative Robotics UVMS Exercise 4

MATLAB controls and integrates the vehicle motion. Unity displays the robot and supplies altitude measurements.

## Run

Requires MATLAB with `udpport` (Instrument Control Toolbox) and the Unity UVMS viewer.

1. Start the Unity viewer, configured for UDP ports **15000**, **15001** and **15003**.
2. Open MATLAB, set this repository root as the current folder, and run:

```matlab
RobustMain
```

Initial pose, goal and simulation duration are set in `RobustMain.m`. Landing clearance is **0.15 m**, set in `tpik/TaskLanding.m`.

## Read the code in this order

1. `RobustMain.m`: initial state, goals, task lists and simulation loop.
2. `tpik/`: each task defines its reference `xdotbar`, Jacobian `J` and internal activation `A`.
3. `ActionManager.m`: matches tasks, blends actions and calls the iCAT solver in priority order.
4. `UvmsSim.m`: integrates the commanded velocities and refreshes transformations in `UvmsModel.m`.
5. `UnityInterface.m` and `SimulationLogger.m`: sensor/visualization exchange and recorded results.

## Task lists and transitions

Read each action list **left to right, highest priority first**:

- **Navigation:** minimum altitude → horizontal attitude → position → full attitude.
- **Landing:** landing altitude → horizontal attitude → position.
- **Unified list:** landing altitude → minimum altitude → horizontal attitude → position → full attitude.

`compileSmoothChange` scans the unified list in order and matches the **same task objects** used in each action. It produces these masks:

```matlab
safe_navigation = [0 1 1 1 1];
landing         = [1 0 1 1 0];
```

A `1` enables that occurrence; a `0` excludes it. Repeated occurrences can represent a task at different priorities; each action must fit the unified order.

When 3D position error falls below **0.1 m**, Main starts the **2 s** transition. Shared occurrences stay at external activation 1, outgoing ones fade down, and incoming ones fade up. During the transition, iCAT uses `Aeffective = externalActivation * A`, leaving the task's internal `A` unchanged. Afterward, it uses the landing list.

## End-to-end loop

**Unity altitude → robot state → task updates → ActionManager/iCAT → vehicle and joint velocities → MATLAB integration/transforms → Unity display and logging → next cycle.**

The supplied `icat/`, `tools/` and `robust_robot/` folders provide solver, math and robot-kinematics helpers. System velocity is `[q_dot; v_nu]`: seven arm rates followed by six vehicle-frame velocity components.

The course template follows Simetti & Casalino, *A Novel Practical Technique to Integrate Inequality Control Objectives and Task Transitions in Priority Based Control* (2016), and Simetti et al., *Task priority control of underwater intervention systems: theory and applications* (2018).
