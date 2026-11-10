classdef Task_altitude < Task

    properties
        feedbackGain = 20        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 20  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 20   % Maximum absolute component of the linear velocity reference [m/s].

        min_alt = 2
        al_delta = 0.5
    end

    methods
        function updateReference(obj, robot)
            
            if isnan(robot.altitude) || robot.altitude < 0
                obj.xdotbar = 0;
                return
            end


            % The robot altitute is given by the Uvms Model itself
            error = (obj.min_alt + obj.al_delta) - robot.altitude;

            command = obj.feedbackGain * error;
            obj.xdotbar = Saturate(command, obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.
            
            linear_vel = [0 0 1] * robot.wTv(1:3, 1:3);
            
            J_q = zeros(1, 7);
            J_linear = linear_vel; 
            J_angular = zeros(1, 3);


            obj.J = [J_q J_linear J_angular];

        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            
            % the next task can modify it only if it is higher than the min
            % plus the delta, so we can avoid chattering
            
            if isnan(robot.altitude) || robot.altitude < 0
               obj.A = 0;
               return
           end

            obj.A = DecreasingBellShapedFunction( ...
                obj.min_alt, ...
                obj.min_alt + obj.al_delta, ...
                0, ...
                1, ...
                robot.altitude);

        end
    end
end
