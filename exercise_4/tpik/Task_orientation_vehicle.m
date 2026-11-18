classdef Task_orientation_vehicle < Task

    properties
        feedbackGain = 20        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 20  % Maximum absolute component of the angular velocity reference [rad/s].
    end

    methods
        
        function updateReference(obj, robot)
            
            [ang_error, ~] = CartesianError(robot.wTgv, robot.wTv);

            command = obj.feedbackGain * ang_error;

            obj.xdotbar = Saturate(command, obj.maxAngularVelocity);
        end

        function updateJacobian(obj, robot)
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.
            
            wRv = robot.wTv(1:3, 1:3);

            obj.J = [zeros(3, 7), zeros(3), wRv];
        end

        function updateActivation(obj, robot)
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            
            % since it is a less then we use the increasing bell shape
            
            obj.A = eye(3);            
        end
    end
end
