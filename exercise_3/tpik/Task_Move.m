classdef Task_Move < Task

    properties
        feedbackGain = 20        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 0.2  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 5   % Maximum absolute component of the linear velocity reference [m/s].
    end

    methods
        function updateReference(obj, robot)
            % The position error between the vehicle and the goal
            position_error = robot.wTgv(1 : 3, 4) - robot.wTv(1:3, 4);
                        
            % we turn the error into a command
            command = obj.feedbackGain * position_error;
            
            % capping the maximum possible command 
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
            
            J_q = zeros(3, 7);
            J_linear_velocity = robot.wTv(1:3, 1:3);
            J_angular_velocity = zeros(3);
            

            % Concatenate according to [arm (3x7) | vehicle linear (3x3) | vehicle angular (3x3)]
            obj.J = [J_q J_linear_velocity J_angular_velocity];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            
            obj.A = eye(3);
        end
    end
end
