function [pEE, pCtr] = Robot_Position(n, q, L)
% Robot_Position   Calculate the positions of the end-effector and intermediate points of a robot manipulator.
%
%   [pEE, pCtr] = Robot_Position(n, q, L) calculates the positions of the end-effector and intermediate points
%   based on the number of links, joint angles, and link lengths.
%
%   Inputs:
%   - n: Number of links in the robot manipulator
%   - q: Joint angles of the manipulator (1 x n matrix)
%   - L: Link lengths of the manipulator (1 x n matrix)
%
%   Outputs:
%   - pEE: Position of the end-effector (2 x 1 vector)
%   - pCtr: Positions of the intermediate points (2 x n matrix)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

d = zeros(1, n);
offset = zeros(1, n);
alpha = zeros(1, n);
pCtr = zeros(2, n);

for i = 1:n
    T = calculateForwardKinematicsDH(i, q(1:i), d(1:i), L(1:i), alpha(1:i), offset(1:i));
    pCtr(:, i) = T(1:2, 4);
end

pEE = pCtr(:, n);

end
