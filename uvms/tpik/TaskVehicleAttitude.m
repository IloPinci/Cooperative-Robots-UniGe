classdef TaskVehicleAttitude < Task

    properties
        feedbackGain = 3
        maxAngularVelocity = 0.2
        wRv
    end

    methods
        function updateReference(obj, robot)
            Rg = robot.wTgv(1:3,1:3);
            obj.wRv = robot.wTv(1:3,1:3);
            w_rho = -VersorLemma(Rg,obj.wRv); % Current-to-goal correction in world coordinates
            omegabar = obj.feedbackGain*w_rho;
            obj.xdotbar = Saturate(omegabar,obj.maxAngularVelocity);
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7),zeros(3,3),obj.wRv];
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end
