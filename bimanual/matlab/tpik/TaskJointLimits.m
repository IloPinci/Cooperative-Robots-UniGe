classdef TaskJointLimits < Task
    %TASKJOINTLIMITS Keep one arm's joint angles away from their position limits.
    %
    % Each joint is one task component. 
    % Near the lower limit it requests positive velocit
    % Bear the upper limit it requests negative velocity. 
    % Activation fades to zero inside the safe range, leaving motion to lower priorities.

    properties
        robot                 % PandaArm whose joint positions are protected.
        armID                 % "left" or "right"; selects the seven controlled joint columns.
        feedbackGain = 3      % Converts joint-position error into a velocity reference [1/s].
        buffer = 0.1          % Safety interval inside each position limit [rad].
    end

    methods
        function obj = TaskJointLimits(robot, armID, taskName)
            %TASKJOINTLIMITS Bind the position-limit objective to one arm.
            %
            % Inputs:
            %   robot    - PandaArm providing q, jlmin and jlmax.
            %   armID    - "left" or "right"; determines the Jacobian column block.
            %   taskName - Descriptive label used in plots and diagnostics.
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            %UPDATEREFERENCE Request joint motion toward the safe interval.
            %
            % Output:
            %   None. Updates obj.xdotbar as a [7x1] joint-velocity reference [rad/s].
            %   The iCAT solver applies the robot's joint-velocity limits.
            q = obj.robot.q;
            obj.xdotbar = zeros(7,1);

            for i = 1:7
                lower = obj.robot.jlmin(i) + obj.buffer;
                upper = obj.robot.jlmax(i) - obj.buffer;

                if q(i) < lower
                    obj.xdotbar(i) = obj.feedbackGain*(lower-q(i));
                elseif q(i) > upper
                    obj.xdotbar(i) = obj.feedbackGain*(upper-q(i));
                end
            end
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Select this arm's seven joint velocities.
            %
            % Output:
            %   None. Updates obj.J as a [7x14] matrix ordered as [left arm; right arm].
            if obj.armID == "left"
                obj.J = [eye(7) zeros(7)];
            else
                obj.J = [zeros(7) eye(7)];
            end
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Activate each joint independently near either limit.
            %
            % Output:
            %   None. Sets diagonal obj.A as a [7x7] matrix with entries in [0,1].
            q = obj.robot.q;
            obj.A = zeros(7);

            for i = 1:7
                lowerActivation = DecreasingBellShapedFunction(obj.robot.jlmin(i),obj.robot.jlmin(i)+obj.buffer,0,1,q(i));
                upperActivation = IncreasingBellShapedFunction(obj.robot.jlmax(i)-obj.buffer,obj.robot.jlmax(i),0,1,q(i));
                obj.A(i,i) = max(lowerActivation,upperActivation);
            end
        end
    end
end
