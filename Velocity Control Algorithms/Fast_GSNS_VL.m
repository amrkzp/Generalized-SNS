function [dq,scaleFactor] = Fast_GSNS_VL(dZMin,dZMax,A_LIM,task)
%   GSNS_VL:Fast redundancy resolution method that uses a rank-one update formula
% and improves the performance and computational efficiency of the basic GSNS algorithms.
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
eps=0.1;
lambda=0.015;
tol = 1e-6;

nJnt = size(task(1).J,2);
nLim = numel(dZMin);
I = eye(nJnt);
Pi = I;
dqi = zeros(nJnt,1);

iTsk = 1;
scaleFactor = 1.0;
Wi = eye(nLim);

nTask = size(task,2);
while(iTsk <= nTask)
    PiPrev = Pi;
    dqiPrev = dqi;

    % get i-th task Jacobian
    Ji = task(iTsk).J;

    % get i-th task velocity
    dxGoali = task(iTsk).F_ast;

    % initialization of each task (w/o saturations and scaling factor)
    inv1 = damped_pinv(Ji*PiPrev,lambda,eps);

    dq1 = inv1*dxGoali;
    dq2 = -inv1*(Ji*dqiPrev);
    dqw = zeros(nJnt,1);
    xi = zeros(nJnt,1);
    Idx = 1;
    dqNi = 0;

    siStar = 0.0;
    dq1Star = dq1;
    dq2Star = dq2;
    dqwStar = dqw;

    Pi = PiPrev - inv1*(Ji*PiPrev);
    PiBar = PiPrev;
    PiHat = Pi;


    limitExceeded = true;


    while (limitExceeded)
        limitExceeded = false;
        % compute a solution without task scale factor
        dq1 = (I - xi*A_LIM(Idx,:))*dq1;
        dq2 = (I - xi*A_LIM(Idx,:))*dq2;
        dqw = dqw + xi*(dqNi - A_LIM(Idx,:)*(dqiPrev+dqw));
        dqi = dqiPrev + dq1 + dq2 + dqw;

        % compute the resulting general velocity
        dzi = A_LIM*dqi;

        if ( any(dzi<(dZMin-tol)) || any(dzi>(dZMax+tol)) )
            limitExceeded = true;
            % compute scale factor
            a = A_LIM*(dq1+dq2);
            b = dzi - a;

            % scalar factor for cartesian constraints
            marginL = dZMin - b;
            marginU = dZMax - b;

            s = zeros(nLim, 1);
            for i=1:nLim
                if Wi(i,i) == 0
                    s(i) = inf;
                else
                    s(i) = FindScaleFactor(marginL(i), marginU(i), a(i));
                    if(s(i) == 1 && (dzi(i) < (dZMin(i) - tol) || dzi(i) > (dZMax(i) + tol)))
                        s(i) = 0;
                    end
                    if(s(i) < tol && (dzi(i) > (dZMin(i) - tol) && dzi(i) < (dZMax(i) + tol)))
                        s(i) = 1;
                    end
                end
            end


            taskScale = min(s);
            [~, Idx] = min(s);
            Wi(Idx, Idx) = 0;

            if (taskScale > siStar)
                siStar = taskScale;
                dq1Star = dq1;
                dq2Star = dq2;
                dqwStar = dqw;
            end

            % saturate the most critical joint/cartesian constraint
            dqNi = min(max(dZMin(Idx), dzi(Idx)), dZMax(Idx));
            [Q1,R1]=qr((A_LIM(Idx,:)*PiHat)',0);
            xi = Q1/(R1');
            [Q2,R2]=qr((A_LIM(Idx,:)*PiBar)',0);
            xiPrev = Q2/(R2');
            PiBar = (I - xiPrev*A_LIM(Idx,:))*PiBar;
            PiHat = (I - xi*A_LIM(Idx,:))*PiHat;

            if (any(svd(Ji*PiBar) < 1e-6))
                dqi = dqiPrev + siStar*(dq1Star + dq2Star) + dqwStar;

                if (iTsk==1)
                    scaleFactor = siStar;
                end

                dzi = A_LIM*dqi;
                if ( any(dzi<(dZMin-tol)) || any(dzi>(dZMax+tol)) )
                    dqi = dqiPrev;
                end

                break;

            end %end svd check

        end %end bounds check

    end %end limit_exceed
    iTsk = iTsk + 1;
end %end tasks
dq = dqi;
end