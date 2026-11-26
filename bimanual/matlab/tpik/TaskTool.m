classdef TaskTool < Task
    %TASKTOOL Six-dimensional Cartesian pose task for one Panda tool.
    %
    % The task is bound to one PandaArm when it is constructed. The arm identifier is retained only to embed the local [6x7] Jacobian in the correct half of the bimanual [6x14] Jacobian.

    properties
        robot                     % PandaArm whose tool pose and Jacobian define this task.
        armID = ""                % "left" or "right"; selects the Jacobian columns in the bimanual control vector.
        feedbackGain = 1           % Proportional gain converting Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 0.3   % Maximum absolute angular-velocity component [rad/s].
        maxLinearVelocity = 0.3    % Maximum absolute linear-velocity component [m/s].
    end

    methods
        function obj = TaskTool(robot, armID, taskName)
            %TASKTOOL Construct a tool task for one side of the bimanual system.
            %
            % Inputs:
            %   robot    - PandaArm controlled by this task.
            %   armID    - String scalar equal to "left" or "right"; determines whether robot.wJt occupies columns 1:7 or 8:14 of obj.J.
            %   taskName - Descriptive string used in plots and diagnostics.
            if ~isa(robot, 'PandaArm')
                error('TaskTool:InvalidRobot', 'robot must be a PandaArm handle.');
            end
            armID = string(armID);
            if ~isscalar(armID) || ~any(armID == ["left", "right"])
                error('TaskTool:InvalidArmID', 'armID must be either "left" or "right".');
            end
            obj.robot = robot;
            obj.armID = armID;
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            %UPDATEREFERENCE Compute a bounded Cartesian feedback velocity.
            %
            % Output:
            %   None. Updates obj.xdotbar in [angular; linear] order, expressed in the world frame.
            % CartesianError returns geometric errors. The feedback gain converts them into velocity references before angular and linear parts are bounded independently.
            [angularError, linearError] = CartesianError(obj.robot.wTg, obj.robot.wTt);
            obj.robot.orientationError = angularError;
            obj.robot.positionError = linearError;
            obj.xdotbar = obj.feedbackGain*[angularError; linearError];
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), obj.maxAngularVelocity);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), obj.maxLinearVelocity);
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Embed the selected arm Jacobian in the fourteen-dimensional control space.
            %
            % Output:
            %   None. Updates obj.J with columns ordered as [left arm; right arm].
            if obj.armID == "left"
                obj.J = [obj.robot.wJt zeros(6,7)];
            else
                obj.J = [zeros(6,7) obj.robot.wJt];
            end
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Keep all six Cartesian components fully active.
            obj.A = eye(6);
        end
    end
end
