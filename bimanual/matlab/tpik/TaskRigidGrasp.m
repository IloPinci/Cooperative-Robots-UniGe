classdef TaskRigidGrasp < Task
    %TASKRIGIDGRASP Keep both arms inferred object poses and velocities consistent.
    %
    % Each arm predicts the object pose from its tool and the captured tTo.
    % The task constrains the difference between the two object velocities.
    % Pose feedback corrects any diff after the grasp is established.

    properties
        leftArm              % PandaArm holding the object's left grasp point.
        rightArm             % PandaArm holding the object's right grasp point.
        feedbackGain = 3     % Converts disagreement between object poses into a correction [1/s].
    end

    methods
        function obj = TaskRigidGrasp(leftArm, rightArm, taskName)
            %TASKRIGIDGRASP Bind the rigid constraint to both arms.
            %
            % Inputs:
            %   leftArm  - Left PandaArm, with its grasp transform captured.
            %   rightArm - Right PandaArm, with its grasp transform captured.
            %   taskName - Descriptive label used in plots and diagnostics.
            obj.leftArm = leftArm;
            obj.rightArm = rightArm;
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            %UPDATEREFERENCE Correct disagreement between the two object poses.
            %
            % Output:
            %   None. Updates [6x1] obj.xdotbar as desired left-minus-right object
            %   velocity in world axes, ordered as [angular; linear].
            % Matching poses give zero reference: both object velocities must agree.
            [angularError,linearError] = CartesianError(obj.rightArm.wTo,obj.leftArm.wTo);
            obj.xdotbar = obj.feedbackGain*[angularError;linearError];
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Map joint velocities to left-minus-right object velocity.
            %
            % Output:
            %   None. Updates obj.J as a [6x14] matrix ordered as [left arm; right arm].
            leftJ = obj.leftArm.computeObjectJacobian();
            rightJ = obj.rightArm.computeObjectJacobian();
            obj.J = [leftJ -rightJ];
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Fully activate the rigid constraint without blending.
            %
            % Output:
            %   None. Sets [6x6] obj.A to identity while this task is in the action.
            obj.A = eye(6);
        end
    end
end
