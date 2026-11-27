classdef TaskObjectMotion < Task
    %TASKOBJECTMOTION Drive the grasped object toward its desired world pose.
    %
    % One arm supplies the object pose and Jacobian. The higher-priority rigid
    % grasp constraint makes the other arm follow the same object motion.

    properties
        robot                     % PandaArm used to represent the object pose.
        armID                     % "left" or "right"; selects the Jacobian column block.
        feedbackGain = 1           % Converts object-pose error into a velocity reference [1/s].
        maxAngularVelocity = 0.3   % Maximum absolute angular-velocity component [rad/s].
        maxLinearVelocity = 0.3    % Maximum absolute linear-velocity component [m/s].
    end

    methods
        function obj = TaskObjectMotion(robot, armID, taskName)
            %TASKOBJECTMOTION Bind object-motion control to one arm's object estimate.
            %
            % Inputs:
            %   robot    - PandaArm with its grasp transform captured before activation.
            %   armID    - "left" or "right"; determines the Jacobian column block.
            %   taskName - Descriptive label used in plots and diagnostics.
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            %UPDATEREFERENCE Request a bounded object velocity toward wTog.
            %
            % Output:
            %   None. Updates [6x1] obj.xdotbar in world axes, ordered as [angular; linear].
            [angularError,linearError] = CartesianError(obj.robot.wTog,obj.robot.wTo);
            obj.xdotbar = obj.feedbackGain*[angularError;linearError];
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3),obj.maxAngularVelocity);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6),obj.maxLinearVelocity);
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Embed the selected arm's object Jacobian in joint space.
            %
            % Output:
            %   None. Updates [6x14] obj.J with columns ordered as [left arm; right arm].
            wJo = obj.robot.computeObjectJacobian();
            if obj.armID == "left"
                obj.J = [wJo zeros(6,7)];
            else
                obj.J = [zeros(6,7) wJo];
            end
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Keep all six object-pose components active.
            obj.A = eye(6);
        end
    end
end
