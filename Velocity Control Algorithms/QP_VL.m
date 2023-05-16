function [dq,s] = QP_VL(dZMin,dZMax,A_LIM,task,alpha)
% solveSNSVelocityQP   Solve the velocity optimization problem using Quadratic Programming (QP).
%
%   [dq, s] = solveSNSVelocityQP(dZMin, dZMax, A_LIM, task, alpha) solves the velocity optimization problem using QP.
%
%   Inputs:
%   dZMin, dZMax: Minimum and maximum allowable velocities shaped by the
%   function ComputeVelBnds
%   A_LIM: Generalized constraints matrix corresponding to the limits.
%   task: Structure containing information about the task to be performed.
%
%   Outputs:
%   dq: Calculated joint velocities.
%   si: Scale factor applied to the solution.
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

nJnt = size(task(1).J,2);
nLim = numel(dZMin);
I = eye(nJnt);

iTsk=1;
% get i-th task Jacobian
J = task(iTsk).J;
% get i-th task acceleration
dxGoal = task(iTsk).F_ast;
    
% problem dimensions
nDecVar = nJnt + 1;  % [dq; -s]


% set the objective function
problem.H = blkdiag(eye(nJnt), 1.0 / alpha);
problem.f = zeros(nDecVar, 1);

% set the constraints:
problem.lb = [];
problem.ub = [];
problem.Aeq = [J, dxGoal];
problem.beq = dxGoal;
problem.Aineq = [-A_LIM,zeros(size(A_LIM,1),1);A_LIM,zeros(size(A_LIM,1),1)];
problem.bineq = [-dZMin;dZMax];

% solve the problem
% problem.options = optimset('Display','off','TolCon',1e-20,'TolFun',1e-20,'TolX',1e-20);
problem.options = optimset('Display','off');

problem.solver = 'quadprog';
[zSoln, ~, exitCode] = quadprog(problem);
dq = zSoln(1:nJnt);
s = 1-zSoln(end);

end