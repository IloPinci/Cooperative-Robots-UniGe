classdef BimanualSim < handle
    %BIMANUALSIM Fixed-step kinematic simulator for two Panda manipulators.
    %
    % The simulator integrates the fourteen joint velocities with an explicit Euler step and then refreshes the forward and differential kinematics stored in both PandaArm objects.

    properties
        leftArm     % PandaArm representing the left manipulator.
        rightArm    % PandaArm representing the right manipulator.
        dt          % Positive integration time step [s].
        time        % Current simulation time [s].
        maxSteps    % Number of integration steps required to reach endTime.
        loopCounter % Number of completed integration steps.
    end

    methods
        function obj = BimanualSim(dt, leftArm, rightArm, endTime)
            %BIMANUALSIM Construct a fixed-step simulator.
            %
            % Inputs:
            %   dt       - Positive integration time step [s].
            %   leftArm  - PandaArm used as the left manipulator.
            %   rightArm - PandaArm used as the right manipulator.
            %   endTime  - Requested simulation duration [s].
            %
            % Output:
            %   obj - Initialized BimanualSim handle object.
            obj.dt = dt;
            obj.leftArm = leftArm;
            obj.rightArm = rightArm;
            obj.time = 0;
            obj.loopCounter = 0;
            obj.maxSteps = ceil(endTime/dt);
            obj.updateKinematics();
        end

        function step(obj, q_dot)
            %STEP Integrate both manipulators by one sample.
            %
            % Input:
            %   q_dot - [14x1] joint velocity vector ordered as [left arm; right arm] [rad/s].
            %
            % Output:
            %   None. Updates both PandaArm objects, their kinematics, obj.time and obj.loopCounter.
            q_dot = q_dot(:);
            if numel(q_dot) ~= 14
                error('BimanualSim:InvalidJointVelocity', 'q_dot must contain seven velocities for each Panda arm.');
            end

            obj.leftArm.q_dot = q_dot(1:7);
            obj.rightArm.q_dot = q_dot(8:14);
            obj.leftArm.q = obj.leftArm.q + obj.leftArm.q_dot*obj.dt;
            obj.rightArm.q = obj.rightArm.q + obj.rightArm.q_dot*obj.dt;
            obj.updateKinematics();

            obj.time = obj.time + obj.dt;
            obj.loopCounter = obj.loopCounter + 1;
        end

        function updateKinematics(obj)
            %UPDATEKINEMATICS Recompute forward and differential kinematics for both arms.
            %
            % Input/output:
            %   None. Updates the transforms and Jacobians stored in obj.leftArm and obj.rightArm.
            obj.leftArm.updateTransform();
            obj.rightArm.updateTransform();
            obj.leftArm.updateJacobian();
            obj.rightArm.updateJacobian();
        end
    end
end
