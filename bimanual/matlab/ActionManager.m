classdef ActionManager < handle
    %ACTIONMANAGER Store named actions and solve the active bimanual task hierarchy.
    %
    % An action is a cell array of Task objects ordered from highest to lowest priority. Actions are registered and selected by descriptive string names rather than numeric indices.

    properties (Access = private)
        actions                              % Dictionary: action name -> prioritized task stack.
        currentActionName (1,1) string = "" % Name of the active action.
        dof                                  % Total number of controlled joints in both arms.
        satMin                               % [dof x 1] original lower joint-velocity limits.
        satMax                               % [dof x 1] original upper joint-velocity limits.
    end

    properties (Constant, Access = private)
        residualLambda = 1e-4
        residualThreshold = 0.01
        residualGain = 10
    end

    methods
        function obj = ActionManager(bimanualSystem)
            %ACTIONMANAGER Construct an empty action library for a bimanual system.
            %
            % Input:
            %   bimanualSystem - BimanualSim whose two arm dimensions define the control-space size.
            %
            % Output:
            %   obj - Initialized ActionManager with no active action.
            obj.actions = configureDictionary("string", "cell");
            obj.dof = numel(bimanualSystem.leftArm.q) + numel(bimanualSystem.rightArm.q);

            leftMax = bimanualSystem.leftArm.q_dot_max(:);
            rightMax = bimanualSystem.rightArm.q_dot_max(:);
            if numel(leftMax) ~= numel(bimanualSystem.leftArm.q) || numel(rightMax) ~= numel(bimanualSystem.rightArm.q) || any(~isfinite([leftMax; rightMax])) || any([leftMax; rightMax] < 0)
                error('ActionManager:InvalidVelocityLimits', 'Each PandaArm must provide one finite, non-negative q_dot_max value for every joint.');
            end
            obj.satMax = [leftMax; rightMax];
            obj.satMin = -obj.satMax;
        end

        function addAction(obj, actionName, taskStack)
            %ADDACTION Register a prioritized task stack under a unique name.
            %
            % Inputs:
            %   actionName - Non-empty string scalar identifying the action.
            %   taskStack  - Non-empty cell array of Task objects ordered from highest to lowest priority.
            actionName = string(actionName);
            if ~isscalar(actionName) || ismissing(actionName) || strlength(strtrim(actionName)) == 0
                error('ActionManager:InvalidActionName', 'Action name must be a non-empty string scalar.');
            end
            actionName = strtrim(actionName);
            if ~iscell(taskStack) || isempty(taskStack) || ~all(cellfun(@(task) isa(task, 'Task'), taskStack))
                error('ActionManager:InvalidTaskStack', 'Task stack must be a non-empty cell array of Task objects.');
            end
            if isKey(obj.actions, actionName)
                error('ActionManager:DuplicateAction', 'An action named "%s" is already registered.', actionName);
            end
            obj.actions(actionName) = {taskStack};
        end

        function q_dot = computeICAT(obj)
            %COMPUTEICAT Compute the joint-velocity references for the active action.
            %
            % Output:
            %   q_dot - [dof x 1] joint velocity ordered as [left arm; right arm] [rad/s].
            if obj.currentActionName == ""
                error('ActionManager:NoActiveAction', 'Select an action before calling computeICAT.');
            end
            tasks = obj.actions{obj.currentActionName};

            for i = 1:length(tasks)
                tasks{i}.updateReference();
                tasks{i}.updateJacobian();
                tasks{i}.updateActivation();
            end

            % Each priority receives only the velocity range left available by the tasks above it. A single scaling factor is applied to every new increment, preserving its direction while enforcing all joint limits.
            q_dot = zeros(obj.dof,1);
            Qp = eye(obj.dof);
            satMinRemaining = obj.satMin;
            satMaxRemaining = obj.satMax;
            K = eye(obj.dof); % Neutral control-space weighting required by the shared saturated iCAT interface.
            for i = 1:length(tasks)
                [Qp, q_dot, ~, ~, ~, satMinRemaining, satMaxRemaining] = iCAT_task(tasks{i}.A, tasks{i}.J, Qp, q_dot, tasks{i}.xdotbar, tasks{i}.lambda, tasks{i}.threshold, tasks{i}.gain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
            end

            % The final residual task minimizes joint velocity only in directions and within velocity bounds left available by the action tasks.
            [~, q_dot] = iCAT_task(eye(obj.dof), eye(obj.dof), Qp, q_dot, zeros(obj.dof,1), obj.residualLambda, obj.residualThreshold, obj.residualGain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
        end

        function setCurrentAction(obj, actionName)
            %SETCURRENTACTION Select a previously registered action by name.
            actionName = string(actionName);
            if ~isscalar(actionName) || ismissing(actionName) || strlength(strtrim(actionName)) == 0
                error('ActionManager:InvalidActionName', 'Action name must be a non-empty string scalar.');
            end
            actionName = strtrim(actionName);
            if ~isKey(obj.actions, actionName)
                error('ActionManager:UnknownAction', 'No action named "%s" has been registered.', actionName);
            end
            obj.currentActionName = actionName;
        end
    end
end
