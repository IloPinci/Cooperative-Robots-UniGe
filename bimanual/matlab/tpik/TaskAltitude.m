classdef TaskAltitude < Task
    %TASKALTITUDE Lower-bound inequality task for one arm's end-effector clearance.
    %
    % Height is measured along world Z above the simulated surface at z = 0.
    % The task is fully active at and below h_min, fades out through the buffer,
    % and is inactive at and above h_min + buffer.
    % Inspired form our original TaskAltitude (for implementation),
    % and TaskTool template for structure

    properties
        robot                    % PandaArm whose end-effector clearance is protected.
        armID                    % "left" or "right"; selects the seven controlled joint columns.
        feedbackGain = 3         % Converts height error into a vertical-velocity reference [1/s].
        maxLinearVelocity = 0.2  % Maximum absolute vertical-velocity reference [m/s].
        buffer = 0.1             % Width of the gradual activation interval [m].
        h_min = 0.15             % Minimum end-effector clearance above the surface [m].
        h                        % Latest clearance, evaluated by updateReference [m].
        e3 = [0;0;1]             % World vertical unit vector.
    end

    methods
        function obj = TaskAltitude(robot, armID, taskName)
            %TASKALTITUDE Bind the clearance objective to one arm.
            %
            % Inputs:
            %   robot    - PandaArm controlled by this task.
            %   armID    - "left" or "right"; determines the Jacobian column block.
            %   taskName - Descriptive label used in plots and diagnostics.
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            %UPDATEREFERENCE Request bounded motion toward h_min + buffer.
            %
            % Output:
            %   None. Updates obj.h and scalar obj.xdotbar, the desired world-Z velocity.
            obj.h = obj.robot.wTe(3,4); % End-effector clearance above the surface at world z = 0
            hdotbar = obj.feedbackGain*(obj.h_min+obj.buffer-obj.h);
            obj.xdotbar = Saturate(hdotbar,obj.maxLinearVelocity);
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Map the fourteen joint velocities to end-effector height rate.
            %
            % Output:
            %   None. Updates obj.J as a [1x14] row ordered as [left arm; right arm].
            %
            % The model configuration includes two gripper joints. Keep the first
            % seven Jacobian columns and the linear-velocity rows (4:6), then
            % rotate from base to world axes and select the vertical component.
            configuration = [obj.robot.q' 0 0];
            bJe = geometricJacobian(obj.robot.robotModel,configuration,'panda_link7');
            wRb = obj.robot.wTb(1:3,1:3);
            Jh = obj.e3'*wRb*bJe(4:6,1:7); % World vertical velocity of the end effector

            if obj.armID == "left"
                obj.J = [Jh zeros(1,7)];
            else
                obj.J = [zeros(1,7) Jh];
            end
        end

        function updateActivation(obj)
            %UPDATEACTIVATION Smoothly activate the minimum-clearance inequality.
            %
            % Output:
            %   None. Sets scalar obj.A in [0,1] using the latest obj.h.
            obj.A = DecreasingBellShapedFunction(obj.h_min,obj.h_min+obj.buffer,0,1,obj.h);
        end
    end
end
