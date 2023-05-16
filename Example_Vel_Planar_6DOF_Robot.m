% Clearing workspace, closing all figures and resetting graphics settings
clc;
clear;
close all;
addpath('Velocity Control Algorithms');
addpath('Robots/Planar_6DOF_robot');
addpath('Results');

%% Initialization
% Initial joint position
angle = pi/6; % (rad)
q1 = [angle;-angle;-angle;2*angle;-angle;-angle]; % Initial joint angles
dq1 = zeros(6,1); % Initial joint velocities
ddq1 = zeros(6,1); % Initial joint accelerations

% Create robot object
robot = Planar_Robot(q1, zeros(6,1)); % Creating robot object
X_A = robot.computeEEPosition*0.99; % Initial end-effector position
X_B = [1;-1]; % Desired end-effector position

% Time parameters
Ts = 0.001; % Sampling time (s)
T = 10; % Duration of the path (s)

% Robot parameters
nJnt = 6; % Number of joints
I = eye(nJnt); % Identity matrix

% Current configuration
q = q1;
dq = dq1;
ddq = ddq1;

% Controller gains
Kp = 5; % first task
Kp2 = 200; % second task

% Initial Cartesian point
[p,dp,ddp] = Task_planar_linear(0, T, X_A, X_B);
desiredAcc = zeros(2,1);
desiredVelocity = zeros(2,1);
desiredPosition = p;
desiredAcc1 = 0;
desiredVelocity1 = 0;

% Bounds
% Joint Limits
jointPosLim = pi/2; % (rad)
QMax = jointPosLim * ones(nJnt, 1);
QMin = -QMax ;

% Joint Velocity Limits
jointVelLim = 0.4; % (rad/s)
VJntMax = jointVelLim * ones(nJnt, 1);
VJntMin = -VJntMax;

% Joint Acceleration Limits
jointAccLim = 1; % (rad/s^2)
AJntMax = jointAccLim * ones(nJnt, 1);
AJntMin = -AJntMax;


% Cartesian constraints for the points of interests
numControlPoints = 6;
ignored = 1000; % large value to ignor the axis

% Position Limits
cartPosLimYMax  = 1; % (m)
cartPosLimYMin = -1.1; % (m)
PMax = [ignored; cartPosLimYMax] * ones(1, numControlPoints); % Maximum position
PMin = [-ignored; cartPosLimYMin] * ones(1, numControlPoints); % Minimum position

% Velocity Limits
CartVelLim = 1; % (m/s)
VCarMax = [ignored; CartVelLim] * ones(1, numControlPoints); % Maximum velocity
VCarMin = -VCarMax; % Minimum velocity

% Acceleration Limits
CartAccLim = 1; % (m/s^2)
ACarMax = [ignored; CartAccLim] * ones(1, numControlPoints); % Maximum acceleration
ACarMin = -ACarMax; % Minimum acceleration

% Reshaping the constraints
PMax = PMax(:);
PMin = PMin(:);
VCarMax = VCarMax(:);
VCarMin = VCarMin(:);
ACarMax = ACarMax(:);
ACarMin = ACarMin(:);

% Generalized constraints
XMax = [QMax; PMax];
XMin = [QMin; PMin];
VXMax = [VJntMax; VCarMax];
VXMin = [VJntMin; VCarMin];
AXMax = [AJntMax; ACarMax];
AXMin = [AJntMin; ACarMin];

% Inverse Kinematics
IK_flag = false;
if (IK_flag)
    q = IK_func2(p, QMin, QMax, dq1);
    robot.set_q_and_qdot(q,dq1);
end

% Preallocate arrays for performance
Size = round(T/Ts);
Q = zeros(Size,nJnt);
dQ = zeros(Size,nJnt);
ddQ = zeros(Size,nJnt);
A_CtrPnts = zeros(Size,numControlPoints*2);
V_CtrPnts = zeros(Size,numControlPoints*2);
P_CtrPnts = zeros(Size,numControlPoints*2);
T0d = zeros(Size,2);
e = zeros(Size,2);
Pe1 = zeros(Size,2);
exc_time_GSNS = zeros(Size,1);
exc_time_FGSNS = zeros(Size,1);
exc_time_QP = zeros(Size,1);
scaleFactor = zeros(Size,1);

%%% Control loop
s = 0; i = 1;
for t = 0:Ts:T
    p_ee = robot.computeEEPosition;
    P_CtrPnt = robot.computeCtrsPosition;
    P_CtrPnt = reshape(P_CtrPnt,numel(P_CtrPnt),1);
    J_ee = robot.computeEEJacobian;
    J_CtrPnt = robot.computeCtrsJacobian;

    [p,dp,ddp] = Task_planar_linear(t, T, X_A, X_B);
    desiredPosition(:,i) = p;
    desiredVelocity = dp;
    desiredAcc = ddp;
    e(i,:) = desiredPosition(:,i) - p_ee;

    % Create the tasks structure
    % First task
    task(1).J = J_ee;
    task(1).F_ast = desiredVelocity + Kp*(e(i,:)');

    % Second task
    task(2).J = I;
    task(2).F_ast = Kp2*(QMax + QMin- 2*q)./(nJnt*2*(QMax - QMin).^2); %joint centering gradient

    % Augmenting the Joint with Cartesian Constraints
    X = cat(1,q,P_CtrPnt);
    A_LIM = cat(1,I,J_CtrPnt);

    % Shaping the velocity bounds
    [dXMin,dXMax] = computeVelocityBounds(X,XMin,XMax,VXMin,VXMax,AXMax,Ts);

    % SNS
    tic
    [dq_GSNS,s_GSNS] = GSNS_VL(dXMin,dXMax,A_LIM,task);
    t_basic_GSNS = toc;
    
    % Fast SNS
    tic
    [dq_FGSNS,s_FGSNS] = Fast_GSNS_VL(dXMin,dXMax,A_LIM,task);
    t_fast_GSNS = toc;

    % QP
    alpha = 1e-3;
    tic
    [dq_QP,s_QP] = QP_VL(dXMin,dXMax,A_LIM,task,alpha);
    t_QP = toc;


    % dq = dq_GSNS;
    % s = s_GSNS;
    dq = dq_FGSNS;
    s = s_FGSNS;
    % dq = dq_QP;
    % s = s_QP;

    robot.set_q_and_qdot(q,dq);
    v_ee = robot.computeEEVelocity;
    V_CtrPnt = robot.computeCtrsVelocity;
    V_CtrPnt = reshape(V_CtrPnt,numel(V_CtrPnt),1);

    % Save current quantities
    Q(i,:) = q;
    dQ(i,:) = dq;
    ddQ(i,:) = ddq;
    scaleFactor(i) = s;
    exc_time_GSNS(i) = t_basic_GSNS;
    exc_time_FGSNS(i) = t_fast_GSNS;
    exc_time_QP(i) = t_QP;
    V_CtrPnts(i,:) = V_CtrPnt;
    P_CtrPnts(i,:) = P_CtrPnt;
    T0d(i,:) = p_ee;

    % Update
    q = q + dq*Ts; %%current position
    robot.set_q_and_qdot(q,dq);
    i = i + 1;

    if (mod(t,1) == 0)
        disp(t);
    end
end
disp('finish');

% Plotting results
plot_results_multiCtrPnts

% disp(mean(exc_time_GSNS)*1000)
% disp(mean(exc_time_QP)*1000)
