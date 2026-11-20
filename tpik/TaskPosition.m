classdef TaskPosition < Task

    properties
        feedbackGain = 1
        maxLinearVelocity = 1.2
    end

    methods
        function updateReference(obj, robot)
            Ogv = robot.wTgv(1:3,4); % Goal vehicle position
            Ov = robot.wTv(1:3,4);  % Vehicle position
            x = Ogv-Ov;            % Displacement in world coordinates

            % Fixed goal: no goal velocity
            xdotbar = -obj.feedbackGain*x;
            obj.xdotbar = Saturate(xdotbar,obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3,1:3);
            obj.J = [zeros(3,7),-wRv,zeros(3,3)];
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end
