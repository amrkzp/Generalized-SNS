function [J_ee, J_ctrPnts] = Robot_Jacobian(n, q, L, epsilon)
% computeRobotJacobian   Compute the Jacobians of the end-effector and intermediate points of a robotic manipulator.
%
%   [J_ee, J_ctrPnts] = computeRobotJacobian(n, q, L, epsilon) computes the Jacobians of the end-effector and intermediate points
%   of a robotic manipulator based on the number of links, joint angles, link lengths, and epsilon value for numerical differentiation.
%
%   Inputs:
%   - n: Number of links in the robotic manipulator
%   - q: Joint angles of the manipulator (n x 1 vector)
%   - L: Link lengths of the manipulator (n x 1 vector)
%   - epsilon: Epsilon value for numerical differentiation (scalar)
%
%   Outputs:
%   - J_ee: Jacobian of the end-effector position (2 x n matrix)
%   - J_ctrPnts: Jacobians of the intermediate points (2n x n matrix)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

epsilon_inv = 1 / epsilon;
[~, pCtr_init] = Robot_Position(n, q, L);

J_ctrTemp = zeros(2, n, n);
for i = 1:n
    f0 = pCtr_init(:, i); % Calculate f0 when no perturbation happens
    for j = 1:n
        qplus = q;
        qplus(j) = q(j) + epsilon;
        [~, pCtr] = Robot_Position(n, qplus, L);
        f1 = pCtr(:, i);
        J_ctrTemp(:, j, i) = (f1 - f0) .* epsilon_inv;
    end
end

J_ctrPnts = zeros(n*2, n);
for i = 1:size(J_ctrTemp, 3)
    J_ctrPnts([2*i-1, 2*i], :) = squeeze(J_ctrTemp(:, :, i));
end

J_ee = J_ctrTemp(:, :, n);

end
