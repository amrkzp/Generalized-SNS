function [pEE, pCtr, J_ee, J_ctrPnts] = Robot_Compute(n, q, L, epsilon)
% computeRobot   Compute the end-effector position, intermediate points positions, and Jacobians of a robotic manipulator.
%
%   [pEE, pCtr, J_ee, J_ctrPnts] = computeRobot(n, q, L, epsilon) computes the end-effector position, intermediate points positions,
%   and Jacobians of a robotic manipulator based on the number of links, joint angles, link lengths, and epsilon value for numerical
%   differentiation.
%
%   Inputs:
%   - n: Number of links in the robotic manipulator
%   - q: Joint angles of the manipulator (n x 1 vector)
%   - L: Link lengths of the manipulator (n x 1 vector)
%   - epsilon: Epsilon value for numerical differentiation (scalar)
%
%   Outputs:
%   - pEE: End-effector position (2 x 1 vector)
%   - pCtr: Positions of the intermediate points (2 x n matrix)
%   - J_ee: Jacobian of the end-effector position (2 x n matrix)
%   - J_ctrPnts: Jacobians of the intermediate points (2n x n matrix)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

[pEE, pCtr] = Robot_Position(n, q, L);
[J_ee, J_ctrPnts] = Robot_Jacobian(n, q, L, epsilon);

end
