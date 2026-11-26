function BimanualMain()
%BIMANUALMAIN Assemble and run the two-Panda task-priority simulation.
%
% The control vector is q_dot = [q_dot_left; q_dot_right]. At every sample the active task hierarchy computes this fourteen-dimensional reference, BimanualSim integrates it, UDPInterface sends the resulting state or command, and SimulationLogger stores the signals required for analysis.
%% Make exercise dependencies visible to MATLAB
% if you have issues, clear the paths from Home -> Set Path to avoid
% ambiguities with other scripts
addpath('./tools');
addpath('./icat');
addpath('./tpik');

%% Configure the simulation
dt = 0.005;
endTime = 20;
realRobot = false;

% Each PandaArm loads the common mechanical model from panda.mat. The two instances differ only through their fixed world-to-base transforms.
wTbLeft = eye(4);
wTbRight = [RPYToRotationMatrix(0,0,pi),[1.06;-0.01;0];0 0 0 1];
leftArm = PandaArm(wTbLeft);
rightArm = PandaArm(wTbRight);
bimanualSystem = BimanualSim(dt, leftArm, rightArm, endTime);

%% Define the object and individual tool goals
objectPosition = [0.5; 0; 0.59];
objectOrientation = RPYToRotationMatrix(0, 0, 0);
objectLength = 0.12;

leftGoalPosition = objectPosition + objectOrientation*[-objectLength/2;0;0];
rightGoalPosition = objectPosition + objectOrientation*[objectLength/2;0;0];

graspRotation = RPYToRotationMatrix(0,deg2rad(30),0);
leftGoalOrientation = leftArm.wTt(1:3,1:3)*graspRotation;
rightGoalOrientation = rightArm.wTt(1:3,1:3)*graspRotation;

leftArm.setGoal(objectPosition,objectOrientation,leftGoalPosition,leftGoalOrientation);
rightArm.setGoal(objectPosition,objectOrientation,rightGoalPosition,rightGoalOrientation);

% The cooperative-motion object goal is stored now for tasks introduced in the following exercise phases.
wTog = [RPYToRotationMatrix(0, 0, 0) [0.65; -0.35; 0.28]; 0 0 0 1];
leftArm.setObjectGoal(wTog);
rightArm.setObjectGoal(wTog);

%% Define named actions as strict-priority task stacks
leftJointLimitsTask = TaskJointLimits(leftArm, "left", "left joint limits");
rightJointLimitsTask = TaskJointLimits(rightArm, "right", "right joint limits");
leftAltitudeTask = TaskAltitude(leftArm, "left", "left altitude");
rightAltitudeTask = TaskAltitude(rightArm, "right", "right altitude");
minimumManipulability = 0.06;
manipulabilityActivationThreshold = 0.07;
leftManipulabilityTask = TaskManipulability(leftArm, "left", "left manipulability", minimumManipulability, manipulabilityActivationThreshold);
rightManipulabilityTask = TaskManipulability(rightArm, "right", "right manipulability", minimumManipulability, manipulabilityActivationThreshold);
leftToolTask = TaskTool(leftArm, "left", "left tool");
rightToolTask = TaskTool(rightArm, "right", "right tool");
goToObjectTasks = {leftJointLimitsTask, rightJointLimitsTask, ...
    leftAltitudeTask, rightAltitudeTask, ...
    leftManipulabilityTask, rightManipulabilityTask, leftToolTask, rightToolTask};

actionManager = ActionManager(bimanualSystem);
actionManager.addAction("go to object", goToObjectTasks);
actionManager.setCurrentAction("go to object");

%% Initialize communication and logging
robotUDP = UDPInterface(realRobot);
logger = SimulationLogger(bimanualSystem, goToObjectTasks);

%% Run the closed-loop kinematic simulation
tic
for step = 1:bimanualSystem.maxSteps
    % In real-robot mode, replace the simulated configuration with the latest measurements before computing the new reference.
    [q_left, q_right] = robotUDP.receive(bimanualSystem.time);
    if realRobot
        bimanualSystem.leftArm.q = q_left;
        bimanualSystem.rightArm.q = q_right;
        bimanualSystem.updateKinematics();
    end

    % Update the active tasks and solve their strict-priority hierarchy.
    q_dot = actionManager.computeICAT();

    % Integrate both arms and refresh their transforms and Jacobians.
    bimanualSystem.step(q_dot);

    % Send either velocity commands to the physical robots or joint positions to PyBullet, then store the completed sample.
    robotUDP.send(bimanualSystem.time, bimanualSystem);
    logger.update(bimanualSystem.time, bimanualSystem.loopCounter);

    if mod(step, round(1/dt)) == 0
        fprintf('t = %.2f s\n', bimanualSystem.time);
    end
    SlowdownToRealtime(dt);
end

%% Review results and release communication resources
logger.plotAll();
delete(robotUDP);
end
