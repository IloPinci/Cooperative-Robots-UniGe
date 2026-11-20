classdef ActionManager < handle
    %ACTIONMANAGER Store actions and solve the active task-priority stack.
    %
    % An action is represented by a cell array of Task objects. The order in the cell array defines strict priority: element 1 has the highest priority, and each following task uses only the motion left available by the tasks above it.
    %
    % The solver uses the UVMS system velocity convention
    %
    %   ydot = [q_dot; v_nu]
    %
    % where q_dot contains the arm joint velocities and v_nu contains the vehicle velocities expressed in the vehicle frame. Their dimensions are obtained from the robot model when the manager is constructed.

    properties (Access = private)
        actions          % Dictionary: action name -> prioritized task stack.
        currentActionName (1,1) string = "" % Name of the active action.
        dofArm           % Number of manipulator degrees of freedom.
        dofVehicle       % Number of vehicle velocity degrees of freedom.
        dof              % Total dimension of ydot = [q_dot; v_nu].
        satMin           % [dof x 1] original lower system-velocity limits.
        satMax           % [dof x 1] original upper system-velocity limits.

        previousActionName (1,1)string = ""
        switchTime = 0

        unifiedTaskList = {}
        isInPrevious = false(1,0)
        isInCurrent = false(1,0)

    end

    properties
        transitionDuration = 3; % duration of ramp
    end

    properties (Constant, Access = private)
        % The residual damping objective is generated internally rather than represented by a Task, so its regularization parameters live here.
        residualLambda = 1e-4
        residualThreshold = 0.01
        residualGain = 10
    end

    methods
        function obj = ActionManager(robot)
            %ACTIONMANAGER Construct an empty action library for a UVMS model.
            %
            % Input:
            %   robot - UvmsModel whose arm and vehicle dimensions and velocity limits define the system velocity vector handled by this manager.
            %
            % Output:
            %   obj - Initialized ActionManager with no active action.
            obj.actions = configureDictionary("string", "cell");
            obj.dofArm = numel(robot.q);
            obj.dofVehicle = numel(robot.v_nu);
            obj.dof = obj.dofArm + obj.dofVehicle;

            armMax = robot.q_dot_max(:);
            vehicleMax = robot.v_nu_max(:);
            if numel(armMax) ~= obj.dofArm || numel(vehicleMax) ~= obj.dofVehicle || any(~isfinite([armMax; vehicleMax])) || any([armMax; vehicleMax] < 0)
                error('ActionManager:InvalidVelocityLimits', 'q_dot_max and v_nu_max must contain one finite, non-negative limit for each arm and vehicle velocity.');
            end
            obj.satMax = [armMax; vehicleMax];
            obj.satMin = -obj.satMax;
        end


        function addAction(obj, actionName, taskStack)
            %ADDACTION Register a prioritized task stack under a unique name.
            %
            % Inputs:
            %   actionName - Non-empty string scalar used to identify the action in subsequent calls.
            %   taskStack - 1-by-N cell array of Task objects, ordered from highest to lowest priority.
            %
            % Output:
            %   None. This method updates obj.actions in place.
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

            % Dictionary values have cell type; the outer cell stores the complete task stack as the single value associated with the key.
            obj.actions(actionName) = {taskStack};
        end

        
        % we have to store the unifing list and we call it after every new
        % action that has been added. 
        function buildUnifyingTaskList(obj, taskList)
            
            % does the list have actual tasks
            if ~iscell(taskList) || isempty(taskList) || ~all(cellfun(@(task) isa(task, 'Task'), taskList))
                error('ActionManager:InvalidTaskList', 'The unifying list must be a non-empty cell array of Task objects.');
            end
            
            % are the ids consistent (not repeated and not empty)
            unifiedIds = ActionManager.taskIds(taskList);
            if any(strlength(unifiedIds) == 0) || numel(unique(unifiedIds)) ~= numel(unifiedIds)
                error('ActionManager:InvalidTaskIds', 'Every task in the unifying list needs a unique, non-empty id.');
            end
            

            actionNames = keys(obj.actions);
            for k = 1:numel(actionNames)
                stack = obj.actions{actionNames(k)};
                [found, positions] = ismember(ActionManager.taskIds(stack), unifiedIds);

                % are all the tasks in the unifing list
                if ~all(found)
                    error('ActionManager:MissingTasks', 'Action "%s" contains tasks that are not in the unifying list.', actionNames(k));
                end

                % are the priorities carrried properly
                if any(diff(positions) <= 0)
                    error('ActionManager:PriorityMismatch', 'The order of the unifying list contradicts the priorities of action "%s".', actionNames(k));
                end
            end

            obj.unifiedTaskList = taskList;
            
            if obj.currentActionName ~= ""
                obj.updateMembership();
            end
        end


        function [v_nu, q_dot] = computeICAT(obj, robot, currentTime)
            %COMPUTEICAT Compute arm and vehicle velocity references.
            %
            % Input:
            %   robot - UvmsModel containing the current state, transforms, goals and sensor data used by the active tasks.
            %
            % Outputs:
            %   v_nu - [dofVehicle x 1] vehicle velocity expressed in the vehicle frame.
            %   q_dot - [dofArm x 1] manipulator joint velocity [rad/s].
            %
            % Each task first updates its reference rate xdotbar, Jacobian J and activation matrix A. The iCAT recursion then processes the tasks in priority order while preserving higher-priority motion.

            if obj.currentActionName == ""
                error('ActionManager:NoActiveAction', 'Select an action before calling computeICAT.');
            end

            if isempty(obj.unifiedTaskList)
                error('ActionManager:NoUnifyingList', 'Call setUnifyingTaskList before computeICAT.');
            end

            % we go through our unified list
            tasks =  obj.unifiedTaskList;
            

            % Refresh all configuration-dependent task quantities before the stack is inverted. A task may use robot state in all three calls.
            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);

                a_p = IncreasingBellShapedFunction(obj.switchTime, ...
                                                   obj.switchTime + obj.transitionDuration, ...
                                                   double(obj.isInPrevious(i)), ...
                                                   double(obj.isInCurrent(i)), ...
                                                   currentTime);
                
                tasks{i}.A = tasks{i}.A * a_p; 
            end

            % Start with zero commanded velocity, the full system motion space and the original velocity bounds. Each priority consumes part of the available velocity range.
            ydotbar = zeros(obj.dof,1);
            Qp = eye(obj.dof);
            satMinRemaining = obj.satMin;
            satMaxRemaining = obj.satMax;
            K = eye(obj.dof); % Neutral control-space weighting required by the current iCAT interface.
            for i = 1:length(tasks)
                [Qp, ydotbar, ~, ~, ~, satMinRemaining, satMaxRemaining] = iCAT_task(tasks{i}.A, tasks{i}.J, Qp, ydotbar, tasks{i}.xdotbar, tasks{i}.lambda, tasks{i}.threshold, tasks{i}.gain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
            end

            % Use any residual degrees of freedom to drive the system velocity toward zero. As the last-priority task, this cannot alter motion already assigned to higher-priority objectives and must respect the velocity range left by the preceding tasks.
            [~, ydotbar] = iCAT_task(eye(obj.dof), eye(obj.dof), Qp, ydotbar, zeros(obj.dof,1), obj.residualLambda, obj.residualThreshold, obj.residualGain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);

            % Split ydotbar using the same ordering used to build every task Jacobian: arm joints first, then the vehicle twist.
            q_dot = ydotbar(1:obj.dofArm);
            v_nu = ydotbar(obj.dofArm+1:obj.dof);
        end


        function setCurrentAction(obj, actionName, time)
            %SETCURRENTACTION Select an action by name.
            %
            % Input:
            %   actionName - String scalar previously passed to addAction.
            %
            % Output:
            %   None. This method updates obj.currentActionName in place.
            actionName = string(actionName);

            if ~isscalar(actionName) || ismissing(actionName) || strlength(strtrim(actionName)) == 0
                error('ActionManager:InvalidActionName', 'Action name must be a non-empty string scalar.');
            end
            actionName = strtrim(actionName);

            if ~isKey(obj.actions, actionName)
                error('ActionManager:UnknownAction', 'No action named "%s" has been registered.', actionName);
            end

            if isempty(obj.unifiedTaskList)
                error('ActionManager:NoUnifyingList', 'Call buildUnifyingTaskList before selecting an action.');
            end

            if actionName == obj.currentActionName
                warning("Don't declare the same action twice!");
                return;
            end


            if obj.currentActionName == ""
                % first execution
                obj.previousActionName = actionName;
            else
                obj.previousActionName = obj.currentActionName;
            end

            obj.currentActionName = actionName;
            obj.switchTime = time;

            % since a new action is set we have to update the membership
            obj.updateMembership();
        end
    end

    % we see if the tasks in the unified list belong in the previous task
    % list and if the belong in the current one. gives us an array with 0
    % or 1 which we then use to determine the min and max of the bell
    % function
    methods (Access = private)
        function updateMembership(obj)
            unifiedIds = ActionManager.taskIds(obj.unifiedTaskList);

            obj.isInPrevious = ismember(unifiedIds, ActionManager.taskIds(obj.actions{obj.previousActionName}));
            obj.isInCurrent = ismember(unifiedIds, ActionManager.taskIds(obj.actions{obj.currentActionName}));
        end
    end

    % for every new action i collect the ids in an array so i can compare
    % with the is member function above
    methods (Static, Access = private)
        function ids = taskIds(taskList)
            ids = strings(1, numel(taskList));

            for i = 1 : numel(taskList)
                ids(i) = taskList{i}.id;
            end
        end
    end

end
