classdef PandaArm < handle
    %PANDAARM Kinematic model of one Franka Emika Panda manipulator.
    %
    % Each instance stores the state and kinematics of one arm. Two PandaArm objects can therefore represent robots with identical mechanics but different fixed base poses wTb in the common world frame.
    %
    % Transform notation follows aTb = ^a T_b: the transform maps coordinates expressed in frame b into frame a. The relevant frames are w (world), b (robot base), e (end effector), t (tool), o (object) and g (goal).

    properties
        robotModel       % rigidBodyTree loaded from panda.mat.
        q                % [7x1] arm joint positions [rad].
        q_dot            % [7x1] arm joint velocities [rad/s].
        wTb              % [4x4] fixed pose of the robot base with respect to the world frame.
        jlmin            % [7x1] lower joint limits [rad].
        jlmax            % [7x1] upper joint limits [rad].
        q_dot_max        % [7x1] maximum absolute joint velocities used by the saturated iCAT recursion [rad/s].
        bTe              % [4x4] end-effector pose with respect to the robot base.
        wTe              % [4x4] end-effector pose with respect to the world frame.
        eTt              % [4x4] fixed tool pose with respect to the end effector.
        wTt              % [4x4] tool pose with respect to the world frame.
        tTo = []         % [4x4] fixed tool-to-object transform; empty until the grasp is captured.
        wTg              % [4x4] desired tool pose with respect to the world frame.
        wTo              % [4x4] object pose in world; inferred from this arm after grasp capture.
        wTog             % [4x4] desired object pose with respect to the world frame.
        wJt              % [6x7] tool Jacobian expressed in world coordinates, ordered as [angular; linear].
        positionError    % [3x1] latest tool-position error expressed in the world frame [m].
        orientationError % [3x1] latest tool-orientation error expressed in the world frame [rad].
    end

    methods
        function obj = PandaArm(wTb)
            %PANDAARM Construct one Panda model at a specified world-frame base pose.
            %
            % Optional input:
            %   wTb - [4x4] fixed pose of the arm base with respect to the world frame. Defaults to eye(4).
            %
            % Output:
            %   obj - Initialized PandaArm handle object.
            if nargin < 1
                wTb = eye(4);
            end

            % Resolve panda.mat relative to this class so construction does not depend on MATLAB's current folder.
            modelFile = fullfile(fileparts(mfilename('fullpath')), 'panda.mat');
            modelData = load(modelFile, 'franka');
            if ~isfield(modelData, 'franka')
                error('PandaArm:MissingRobotModel', 'The file "%s" does not contain the expected variable "franka".', modelFile);
            end
            obj.robotModel = modelData.franka;
            obj.wTb = wTb;

            % Default configuration and velocity used at simulation start.
            obj.q = [0.0167305; -0.762614; -0.0207622; -2.34352; -0.0305686; 1.53975; 0.753872];
            obj.q_dot = zeros(7,1);

            % Mechanical limits supplied by the Panda specification.
            obj.jlmin = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
            obj.jlmax = [2.8973; 1.7628; 2.8973; -0.0698; 2.8973; 3.7525; 2.8973];
            obj.q_dot_max = 2*ones(7,1);

            % Fixed tool mounting relative to panda_link7, as specified in the assignment.
            obj.eTt = [RPYToRotationMatrix(0,0,deg2rad(-44.9949)),[0;0;0.2104];0 0 0 1];
            obj.wTg = eye(4);
            obj.wTo = eye(4);
            obj.wTog = eye(4);
            obj.positionError = zeros(3,1);
            obj.orientationError = zeros(3,1);

            obj.updateTransform();
            obj.updateJacobian();
        end

        function setGoal(obj, objectPosition, objectOrientation, toolGoalPosition, toolGoalOrientation)
            %SETGOAL Store the current object pose and desired tool pose in world coordinates.
            %
            % Inputs:
            %   objectPosition      - [3x1] object position expressed in the world frame [m].
            %   objectOrientation   - [3x3] object orientation expressed in the world frame.
            %   toolGoalPosition    - [3x1] desired tool position expressed in the world frame [m].
            %   toolGoalOrientation - [3x3] desired tool orientation expressed in the world frame.
            %
            % Output:
            %   None. Updates obj.wTo and obj.wTg.
            obj.wTo = [objectOrientation objectPosition; 0 0 0 1];
            obj.wTg = [toolGoalOrientation toolGoalPosition; 0 0 0 1];
        end

        function setObjectGoal(obj, wTog)
            %SETOBJECTGOAL Store the desired object pose.
            %
            % Input:
            %   wTog - [4x4] desired object pose with respect to the world frame.
            %
            % Output:
            %   None. Updates obj.wTog.
            obj.wTog = wTog;
        end

        function captureGrasp(obj)
            %CAPTUREGRASP Store the object pose relative to the current tool frame.
            %
            % Call once the tool has reached its grasp goal. The saved transform
            % stays fixed -> later motion updates wTo through the tool pose.
            obj.tTo = obj.wTt \ obj.wTo;
        end

        function updateTransform(obj)
            %UPDATETRANSFORM Recompute the end-effector and tool poses from obj.q.
            %
            % Input/output:
            %   None. Updates obj.bTe, obj.wTe and obj.wTt, and obj.wTo after grasp capture.
            configuration = [obj.q' 0 0]; % panda.mat includes the two gripper joints after the seven arm joints.
            obj.bTe = getTransform(obj.robotModel, configuration, 'panda_link7');
            obj.wTe = obj.wTb*obj.bTe;
            obj.wTt = obj.wTe*obj.eTt;

            if ~isempty(obj.tTo)
                obj.wTo = obj.wTt*obj.tTo;
            end
        end

        function updateJacobian(obj)
            %UPDATEJACOBIAN Recompute the tool Jacobian in world coordinates.
            %
            % Input/output:
            %   None. Updates obj.wJt as a [6x7] matrix ordered as [angular velocity; linear velocity].
            obj.wJt = obj.computeToolJacobian(obj.q);
        end

        function wJt = computeToolJacobian(obj, q)
            %COMPUTETOOLJACOBIAN Evaluate the world-frame tool Jacobian at an arbitrary arm configuration.
            %
            % Input:
            %   q - [7x1] arm joint configuration [rad].
            %
            % Output:
            %   wJt - [6x7] tool Jacobian expressed in world coordinates and ordered as [angular; linear].
            %
            % This method does not modify obj.q or any stored transformation, allowing it to be used safely for numerical differentiation.
            q = q(:);
            if numel(q) ~= 7 || any(~isfinite(q))
                error('PandaArm:InvalidConfiguration', 'q must contain seven finite joint coordinates.');
            end

            configuration = [q' 0 0];
            bTeAtQ = getTransform(obj.robotModel, configuration, 'panda_link7');
            wTeAtQ = obj.wTb*bTeAtQ;
            bJeAtQ = geometricJacobian(obj.robotModel, configuration, 'panda_link7');

            % Shift the Jacobian from the end-effector origin to the tool origin, then rotate angular and linear components from base axes into world axes.
            Ste = [eye(3) zeros(3); -skew(wTeAtQ(1:3,1:3)*obj.eTt(1:3,4)) eye(3)];
            wRbBlock = [obj.wTb(1:3,1:3) zeros(3); zeros(3) obj.wTb(1:3,1:3)];
            wJt = Ste*wRbBlock*bJeAtQ(:,1:7);
        end

        function dJdq = computeJacobianDerivatives(obj, deltaQ)
            %COMPUTEJACOBIANDERIVATIVES Numerically differentiate the tool Jacobian with respect to every arm joint.
            %
            % Optional input:
            %   deltaQ - Positive joint perturbation used by the forward finite difference [rad]. Defaults to 1e-6.
            %
            % Output:
            %   dJdq - [6x7x7] tensor whose slice dJdq(:,:,k) is the derivative of obj.wJt with respect to q(k).
            %
            % The nominal and perturbed Jacobians are evaluated without changing obj.q, obj.wTt or obj.wJt. Unlike the original C++ implementation, all seven joint derivatives are evaluated because a non-zero end-effector-to-tool offset can make the final joint affect the tool Jacobian.
            if nargin < 2
                deltaQ = 1e-6;
            end
            if ~isscalar(deltaQ) || ~isfinite(deltaQ) || deltaQ <= 0
                error('PandaArm:InvalidJacobianStep', 'deltaQ must be a positive finite scalar.');
            end

            nominalQ = obj.q(:);
            nominalJacobian = obj.computeToolJacobian(nominalQ);
            numberOfJoints = numel(nominalQ);
            dJdq = zeros(size(nominalJacobian,1), size(nominalJacobian,2), numberOfJoints);
            for jointIndex = 1:numberOfJoints
                perturbedQ = nominalQ;
                perturbedQ(jointIndex) = perturbedQ(jointIndex) + deltaQ;
                dJdq(:,:,jointIndex) = (obj.computeToolJacobian(perturbedQ) - nominalJacobian)/deltaQ;
            end
        end

        function [manipulability, manipulabilityJacobian, dJdq] = computeManipulability(obj, deltaQ, lambda, threshold)
            %COMPUTEMANIPULABILITY Evaluate Yoshikawa's manipulability and its joint-space gradient.
            %
            % Optional inputs:
            %   deltaQ   - Positive perturbation used for the numerical Jacobian derivatives [rad]. Defaults to 1e-6.
            %   lambda   - Maximum damping used by RegPseudoInverse. Defaults to 1e-4.
            %   threshold - Singular-value threshold below which damping is introduced. Defaults to 1e-4.
            %
            % Outputs:
            %   manipulability         - Scalar Yoshikawa manipulability, computed as the product of the singular values of the [6x7] tool Jacobian.
            %   manipulabilityJacobian - [1x7] gradient d(manipulability)/dq.
            %   dJdq                   - [6x7x7] numerical Jacobian-derivative tensor used in the computation.
            if nargin < 2
                deltaQ = 1e-6;
            end
            if nargin < 3
                lambda = 1e-4;
            end
            if nargin < 4
                threshold = 1e-4;
            end

            J = obj.computeToolJacobian(obj.q);
            dJdq = obj.computeJacobianDerivatives(deltaQ);
            manipulability = prod(svd(J));
            regularizedInverse = RegPseudoInverse(J, lambda, threshold);
            manipulabilityJacobian = zeros(1,numel(obj.q));
            for jointIndex = 1:numel(obj.q)
                manipulabilityJacobian(jointIndex) = manipulability*trace(dJdq(:,:,jointIndex)*regularizedInverse);
            end
        end
    end
end
