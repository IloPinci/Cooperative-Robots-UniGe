classdef TaskManipulability < Task
    %TASKMANIPULABILITY Lower-bound inequality task for arm manipulability.
    %
    % The scalar task keeps Yoshikawa's manipulability above minimumManipulability. It is fully active at and below that minimum, decreases smoothly through the safety interval and is inactive at and above activationThreshold.

    properties
        robot                       % PandaArm whose manipulability is protected.
        armID = ""                  % "left" or "right"; selects the gradient columns in the bimanual control vector.
        minimumManipulability       % Lower bound below which the task is fully active.
        activationThreshold         % Upper end of the activation interval; must exceed minimumManipulability.
        feedbackGain = 1            % Proportional gain converting manipulability error into a desired manipulability rate [1/s].
        jacobianDerivativeStep = 1e-6       % Joint perturbation used to compute dJdq numerically [rad].
        gradientRegularizationLambda = 1e-4 % Maximum damping used while computing the manipulability gradient.
        gradientSingularValueThreshold = 1e-4 % Singular-value threshold used while computing the manipulability gradient.
        currentManipulability = NaN          % Latest manipulability value evaluated by updateReference.
        localManipulabilityJacobian = []     % [1x7] latest gradient with respect to the selected arm joints.
    end

    methods
        function obj = TaskManipulability(robot, armID, taskName, minimumManipulability, activationThreshold)
            %TASKMANIPULABILITY Construct a lower-bound manipulability task for one arm.
            %
            % Inputs:
            %   robot                 - PandaArm controlled by this task.
            %   armID                 - String scalar equal to "left" or "right"; determines whether the local gradient occupies columns 1:7 or 8:14 of obj.J.
            %   taskName              - Descriptive string used in plots and diagnostics.
            %   minimumManipulability - Non-negative lower bound at which the task becomes fully active.
            %   activationThreshold   - Value above the lower bound at which the task becomes fully inactive.
            if ~isa(robot, 'PandaArm')
                error('TaskManipulability:InvalidRobot', 'robot must be a PandaArm handle.');
            end
            armID = string(armID);
            if ~isscalar(armID) || ~any(armID == ["left", "right"])
                error('TaskManipulability:InvalidArmID', 'armID must be either "left" or "right".');
            end
            if ~isscalar(minimumManipulability) || ~isfinite(minimumManipulability) || minimumManipulability < 0
                error('TaskManipulability:InvalidMinimum', 'minimumManipulability must be a finite, non-negative scalar.');
            end
            if ~isscalar(activationThreshold) || ~isfinite(activationThreshold) || activationThreshold <= minimumManipulability
                error('TaskManipulability:InvalidActivationThreshold', 'activationThreshold must be finite and greater than minimumManipulability.');
            end

            obj.robot = robot;
            obj.armID = armID;
            obj.taskName = string(taskName);
            obj.minimumManipulability = minimumManipulability;
            obj.activationThreshold = activationThreshold;
        end

        function updateReference(obj)
            %UPDATEREFERENCE Evaluate manipulability and request motion toward the safe region.
            %
            % Output:
            %   None. Updates obj.currentManipulability, obj.localManipulabilityJacobian and the scalar reference rate obj.xdotbar.
            [obj.currentManipulability, obj.localManipulabilityJacobian] = obj.robot.computeManipulability(obj.jacobianDerivativeStep, obj.gradientRegularizationLambda, obj.gradientSingularValueThreshold);

            % The reference is positive only below the upper activation threshold. The iCAT joint-velocity limits bound the resulting joint motion.
            manipulabilityError = max(obj.activationThreshold - obj.currentManipulability, 0);
            obj.xdotbar = obj.feedbackGain*manipulabilityError;
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Embed the local manipulability gradient in the fourteen-dimensional control space.
            %
            % Output:
            %   None. Updates obj.J as a [1x14] row ordered as [left arm; right arm].
            if obj.armID == "left"
                obj.J = [obj.localManipulabilityJacobian zeros(1,7)];
            else
                obj.J = [zeros(1,7) obj.localManipulabilityJacobian];
            end
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Smoothly activate the lower-bound inequality.
            %
            % Output:
            %   None. Sets scalar obj.A to one below the minimum, zero above the activation threshold and a cosine transition between them.
            obj.A = DecreasingBellShapedFunction(obj.minimumManipulability, obj.activationThreshold, 0, 1, obj.currentManipulability);
        end
    end
end
