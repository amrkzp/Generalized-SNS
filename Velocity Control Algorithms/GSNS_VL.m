function [dq,scaleFactor] = GSNS_VL(dZMin,dZMax,A_LIM,task)
%   GSNS_VL: Function implementing the Generalized Saturation in the Null Space (SNS) method for velocity-level tasks.
%   The function calculates the joint velocities that will achieve the desired end-effector velocity for the task,
%   while avoiding singularities and respecting the generalized joint and Cartesian limits.
%
%   Inputs:
%   dZMin, dZMax: Minimum and maximum allowable velocities shaped by the
%   computeVelocityBounds function
%   A_LIM: Generalized constraints matrix corresponding to the limits.
%   task: Structure containing information about the task to be performed.
%
%   Outputs:
%   dq: Calculated joint velocities.
%   scaleFactor: Scale factor applied to the solution.
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

% Tolerance and damping parameters
epsilon = 0.1;
dampingFactor = 0.001;
tolerance = 1e-6;

% Initialization
numTask = size(task, 2);
numJoints = size(task(1).J, 2);
numLimits = numel(dZMin);
I = eye(numJoints);
Z = zeros(numJoints);
Pi = I;
dqi = zeros(numJoints, 1);

% Main loop over tasks
taskIdx = 1;
scaleFactor = 1.0;
while(taskIdx <= numTask)
    % get i-th task Jacobian
    Ji = task(taskIdx).J;
    % get i-th task velocity
    dxTaski = task(taskIdx).F_ast;

    % update variables for previous projection matrix and solution
    PiPrev = Pi;
    dqiPrev = dqi;

    % initialize variables for i-th task
    dqNulli = zeros(size(A_LIM,1),1);
    Ai_LIM = zeros(size(A_LIM));
    Ai_limStar = zeros(size(A_LIM));
    Wi = eye(numLimits);
    PiBar = PiPrev;
    PiBarStar = PiPrev;
    siStar = 0.0;
    dqNulliStar = zeros(size(A_LIM,1),1);
    limitExceeded = true;
    cntLoop = 1;
    while (limitExceeded)
        limitExceeded = false;
        % compute a solution without task scale factor
        inv = damped_pinv(Ji*PiBar,dampingFactor,epsilon); % damped pseudo-inverse solution
        dqHati = inv*(dxTaski - Ji*dqiPrev);

        % Update joint velocities
        if (cntLoop == 1) % first interation
            dqi = dqiPrev + dqHati;
        else % rest of the interations
            PTilde = (I-inv*Ji)*pinv(Ai_LIM*PiPrev);
            v_sat = dqNulli(:,1) - Ai_LIM*dqiPrev;
            dqi = dqiPrev + dqHati + PTilde*v_sat;
        end

        % compute the resulting general velocities
        dzi = A_LIM*dqi;

        % check whether the solution violates the limits
        if ( any(dzi<(dZMin-tolerance)) || any(dzi>(dZMax+tolerance)) )

            limitExceeded = true;
            % compute scale factor
            a = A_LIM*inv*(dxTaski - Ji*dqiPrev);
            b = dzi - a;

            % scalar factor for cartesian constraints
            marginL = dZMin - b;
            marginU = dZMax - b;

            s = zeros(numLimits, 1);
            for i=1:numLimits
                if Wi(i,i) == 0
                    s(i) = inf;
                else
                    s(i) = FindScaleFactor(marginL(i), marginU(i), a(i));
                    if(s(i) == 1 && (dzi(i) < (dZMin(i) - tolerance) || dzi(i) > (dZMax(i) + tolerance)))
                        s(i) = 0;
                    end
                    if(s(i) < tolerance && (dzi(i) > (dZMin(i) - tolerance) && dzi(i) < (dZMax(i) + tolerance)))
                        s(i) = 1;
                    end
                end
            end

            % Compute task scale factor and its index
            taskScale = min(s);
            [~, Idx] = min(s);

            % If task scale factor is infinite, return previous solution
            if (isinf(taskScale))
                dqi = dqiPrev;
                break;
            end

            % update best scaling solution
            if (taskScale > siStar)
                siStar = taskScale;
                Ai_limStar = Ai_LIM;
                dqNulliStar = dqNulli;
                PiBarStar = PiBar;
            end

            % saturate the most critical joint/cartesian constraint
            Ai_LIM(cntLoop,:) = A_LIM(Idx,:);
            dqNulli(cntLoop) = min(max(dZMin(Idx), dzi(Idx)), dZMax(Idx));
            Wi(Idx, Idx) = 0;

            % update projection matrices
            PiBar = PiPrev - pinv(Ai_LIM*PiPrev)*Ai_LIM*PiPrev;

            % If projection matrix is singular, return best scaling solution
            if (any(svd(Ji*PiBar) < tolerance))
                inv = damped_pinv(Ji*PiBarStar,dampingFactor,epsilon);
                PTilde = (I-inv*Ji)*pinv(Ai_limStar*PiPrev);
                v_sat = dqNulliStar(:,1) - Ai_limStar*dqiPrev;
                dqi = dqiPrev + inv*siStar*(dxTaski - Ji*dqiPrev) + PTilde*v_sat;
                if (taskIdx==1)
                    scaleFactor = siStar;
                end

                dzi = Ai_LIM*dqi;
                if ( any(dzi<(dZMin-tolerance)) || any(dzi>(dZMax+tolerance)) )
                    dqi = dqiPrev;
                end

                break;
            end
        end
        cntLoop = cntLoop + 1;
    end

    Pi = PiPrev - damped_pinv(Ji*PiPrev,dampingFactor,epsilon)*(Ji*PiPrev);
    taskIdx = taskIdx + 1;
end
% Return calculated joint velocities
dq = dqi;
end