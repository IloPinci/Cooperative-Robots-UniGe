classdef SimulationLogger < handle
    %SIMULATIONLOGGER Preallocate, store and plot bimanual simulation signals.
    %
    % The logger observes a BimanualSim handle and one explicit prioritized task stack. It does not inspect the private action dictionary maintained by ActionManager.

    properties
        t                % [1 x N] simulation time samples [s].
        q_left           % [7 x N] left-arm joint positions [rad].
        q_dot_left       % [7 x N] left-arm joint velocities [rad/s].
        q_right          % [7 x N] right-arm joint positions [rad].
        q_dot_right      % [7 x N] right-arm joint velocities [rad/s].
        activation_task  % [T x N] cell array containing task activation diagonals.
        xdotbar_task     % [T x N] cell array containing task reference-rate vectors.
        bimanualSystem   % BimanualSim handle read during logging.
        taskSet          % Cell array of Task handles monitored by this logger.
    end

    methods
        function obj = SimulationLogger(bimanualSystem, taskSet)
            %SIMULATIONLOGGER Allocate storage for the complete simulation.
            %
            % Inputs:
            %   bimanualSystem - BimanualSim handle whose two arm states will be logged.
            %   taskSet        - Cell array of Task objects to monitor.
            %
            % Output:
            %   obj - Initialized SimulationLogger handle object.
            obj.bimanualSystem = bimanualSystem;
            obj.taskSet = taskSet;
            maxSamples = bimanualSystem.maxSteps;
            numberOfTasks = length(taskSet);

            obj.t = zeros(1,maxSamples);
            obj.q_left = zeros(7,maxSamples);
            obj.q_dot_left = zeros(7,maxSamples);
            obj.q_right = zeros(7,maxSamples);
            obj.q_dot_right = zeros(7,maxSamples);
            obj.activation_task = cell(numberOfTasks,maxSamples);
            obj.xdotbar_task = cell(numberOfTasks,maxSamples);
        end

        function update(obj, time, sampleIndex)
            %UPDATE Store one state and task sample.
            %
            % Inputs:
            %   time        - Current simulation time [s].
            %   sampleIndex - One-based destination column in the preallocated arrays.
            %
            % Output:
            %   None. Writes into the logger arrays in place.
            obj.t(sampleIndex) = time;
            obj.q_left(:,sampleIndex) = obj.bimanualSystem.leftArm.q;
            obj.q_dot_left(:,sampleIndex) = obj.bimanualSystem.leftArm.q_dot;
            obj.q_right(:,sampleIndex) = obj.bimanualSystem.rightArm.q;
            obj.q_dot_right(:,sampleIndex) = obj.bimanualSystem.rightArm.q_dot;

            for i = 1:length(obj.taskSet)
                obj.activation_task{i,sampleIndex} = diag(obj.taskSet{i}.A);
                obj.xdotbar_task{i,sampleIndex} = obj.taskSet{i}.xdotbar;
            end
        end

        function plotAll(obj, taskIndices)
            %PLOTALL Plot both arm states and selected task reference rates.
            %
            % Optional input:
            %   taskIndices - Indices of tasks in obj.taskSet to plot. Defaults to every monitored task.
            %
            % Output:
            %   None. Creates MATLAB figures from the logged arrays.
            if nargin < 2
                taskIndices = 1:length(obj.taskSet);
            end

            figure(1);
            subplot(2,1,1);
            plot(obj.t, obj.q_left', 'LineWidth', 1);
            title('Left arm joint positions');
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot_left', 'LineWidth', 1);
            title('Left arm joint velocities');
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            figure(2);
            subplot(2,1,1);
            plot(obj.t, obj.q_right', 'LineWidth', 1);
            title('Right arm joint positions');
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot_right', 'LineWidth', 1);
            title('Right arm joint velocities');
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            figure(3);
            for i = 1:length(taskIndices)
                taskIndex = taskIndices(i);
                subplot(length(taskIndices),1,i);
                referenceHistory = cell2mat(obj.xdotbar_task(taskIndex,:));
                plot(obj.t, referenceHistory', 'LineWidth', 1);
                grid on;
                title("Task reference: " + obj.taskSet{taskIndex}.taskName);
                if size(referenceHistory,1) == 6
                    legend('omega_x','omega_y','omega_z','v_x','v_y','v_z');
                end
            end
        end
    end
end
