classdef UDPInterface < handle
    %UDPINTERFACE Exchange bimanual state and commands over UDP.
    %
    % In real-robot mode the interface receives the measured configuration of each Panda and sends the corresponding joint-velocity reference. In simulation mode it sends both simulated configurations to the PyBullet environment.

    properties (Access = private)
        leftReceiver   % UDP receiver for the left-arm measured configuration.
        rightReceiver  % UDP receiver for the right-arm measured configuration.
        leftSender     % UDP sender for the left-arm velocity command.
        rightSender    % UDP sender for the right-arm velocity command.
        pybulletSender % UDP sender for the simulated configuration.
        realRobot      % True when communicating with the physical robot setup.
        lastQLeft      % [7x1] latest valid left-arm configuration [rad].
        lastQRight     % [7x1] latest valid right-arm configuration [rad].
    end

    methods
        function obj = UDPInterface(realRobot)
            %UDPINTERFACE Configure the UDP channels required by the selected setup.
            %
            % Optional input:
            %   realRobot - Logical scalar. True selects the physical-robot channels; false selects the PyBullet channel. Defaults to false.
            %
            % Output:
            %   obj - Initialized UDPInterface handle object.
            if nargin < 1
                realRobot = false;
            end
            obj.realRobot = logical(realRobot);
            obj.lastQLeft = zeros(7,1);
            obj.lastQRight = zeros(7,1);

            if obj.realRobot
                obj.leftReceiver = dsp.UDPReceiver('LocalIPPort', 1501, 'MaximumMessageLength', 255);
                obj.rightReceiver = dsp.UDPReceiver('LocalIPPort', 1503, 'MaximumMessageLength', 255);
                obj.leftSender = dsp.UDPSender('RemoteIPPort', 1500, 'RemoteIPAddress', '127.0.0.1');
                obj.rightSender = dsp.UDPSender('RemoteIPPort', 1502, 'RemoteIPAddress', '127.0.0.1');
            else
                obj.pybulletSender = dsp.UDPSender('RemoteIPPort', 1505, 'RemoteIPAddress', '127.0.0.1');
            end
        end

        function [q_left, q_right] = receive(obj, time)
            %RECEIVE Return the latest measured joint configurations.
            %
            % Input:
            %   time - Current simulation time [s]. At time zero the real-robot mode waits for the first complete pair of measurements.
            %
            % Outputs:
            %   q_left  - [7x1] latest left-arm configuration [rad].
            %   q_right - [7x1] latest right-arm configuration [rad].
            if ~obj.realRobot
                q_left = obj.lastQLeft;
                q_right = obj.lastQRight;
                return
            end

            leftData = step(obj.leftReceiver);
            rightData = step(obj.rightReceiver);
            if time == 0
                while isempty(leftData)
                    leftData = step(obj.leftReceiver);
                    pause(0.001);
                end
                while isempty(rightData)
                    rightData = step(obj.rightReceiver);
                    pause(0.001);
                end
            end

            if ~isempty(leftData)
                obj.lastQLeft = typecast(leftData, 'double');
                obj.lastQLeft = obj.lastQLeft(:);
            end
            if ~isempty(rightData)
                obj.lastQRight = typecast(rightData, 'double');
                obj.lastQRight = obj.lastQRight(:);
            end
            q_left = obj.lastQLeft;
            q_right = obj.lastQRight;
        end

        function send(obj, time, bimanualSystem)
            %SEND Transmit velocity commands or simulated joint positions.
            %
            % Inputs:
            %   time           - Current simulation time [s].
            %   bimanualSystem - BimanualSim containing the current state and applied joint velocities.
            %
            % Output:
            %   None. Sends one datagram per configured output channel.
            if obj.realRobot
                step(obj.leftSender, [time; bimanualSystem.leftArm.q_dot]);
                step(obj.rightSender, [time; bimanualSystem.rightArm.q_dot]);
            else
                step(obj.pybulletSender, [bimanualSystem.leftArm.q; bimanualSystem.rightArm.q]);
            end
        end

        function delete(obj)
            %DELETE Release any configured UDP System objects.
            try
                if obj.realRobot
                    release(obj.leftReceiver);
                    release(obj.rightReceiver);
                    release(obj.leftSender);
                    release(obj.rightSender);
                else
                    release(obj.pybulletSender);
                end
            catch
            end
        end
    end
end
