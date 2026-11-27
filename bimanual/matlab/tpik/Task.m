classdef Task < handle
    %TASK Abstract interface for one objective in the bimanual iCAT hierarchy.
    %
    % Every concrete task computes a reference rate xdotbar, a Jacobian J and an activation matrix A satisfying xdot = J*q_dot. For the bimanual system, q_dot is ordered as [left arm; right arm].

    properties
        xdotbar = []       % [m x 1] desired task-space rate.
        J = []             % [m x 14] task Jacobian.
        A = []             % [m x m] diagonal task activation matrix.
        taskName = ""      % Descriptive name used by the logger.
        lambda = 1e-4      % Maximum singular-value-oriented damping.
        threshold = 0.01   % Singular-value threshold below which damping is introduced.
        gain = 10          % Weight protecting directions constrained by higher-priority tasks.
    end

    methods
        function obj = Task(lambda, threshold, gain)
            %TASK Optionally override the default iCAT regularization parameters.
            if nargin >= 1
                obj.lambda = lambda;
            end
            if nargin >= 2
                obj.threshold = threshold;
            end
            if nargin >= 3
                obj.gain = gain;
            end
        end
    end

    methods (Abstract)
        updateReference(obj)
        updateJacobian(obj)
        updateActivation(obj)
    end
end
