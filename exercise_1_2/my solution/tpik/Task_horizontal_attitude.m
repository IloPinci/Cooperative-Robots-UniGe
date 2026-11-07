classdef Task_horizontal_attitude < Task

    properties
        feedbackGain = 20        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 20  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].

        theta_max = 0.2
        current_theta = 0
        delta = 0.1
        world_rho = zeros(3, 1)
    end

    methods
        function updateReference(obj, robot)
            k_world = [0; 0 ; 1];           % the world axis
            k_vehicle = robot.wTv(1:3, 3);        % the vehicle axis expressed in the world frame
            
            % we get angle axis vector which takes the world's z-axis onto
            % the vehicle's z axis and projects that into the world frame.
            % The norma of that result is the angle of rotation theta
            obj.world_rho = ReducedVersorLemma(k_world, k_vehicle);
            obj.current_theta = norm(obj.world_rho);
            
            % we compute the difference and the corresponding command
            difference = obj.theta_max - obj.current_theta;
            command = obj.feedbackGain * difference;

            obj.xdotbar = Saturate(command, obj.maxAngularVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.
            
            wRv = robot.wTv(1:3, 1:3);
            
            if obj.current_theta ~= 0
                world_axis = obj.world_rho / obj.current_theta;  % the agle axis vector. Here we get the axis
                obj.J = world_axis' * [zeros(3, 7), zeros(3), wRv];
            else
                % this is the case where they are aligned 
                obj.J = zeros(1, 13);
            end

        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            
            % since it is a less then we use the increasing bell shape
            obj.A = IncreasingBellShapedFunction( ...
                obj.theta_max - obj.delta, ...
                obj.theta_max, ...
                0, ...
                1, ...
                obj.current_theta);

            
        end
    end
end
