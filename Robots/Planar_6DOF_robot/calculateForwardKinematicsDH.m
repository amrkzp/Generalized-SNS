function T = calculateForwardKinematicsDH(numLinks, jointAngles, dParams, aParams, alphaParams, offsetParams)
% calculateForwardKinematicsDH   Calculate forward kinematics of a robotic manipulator using Denavit-Hartenberg parameters.
%
%   T = calculateForwardKinematicsDH(numLinks, jointAngles, dParams, aParams, alphaParams, offsetParams) calculates the 
%   transformation matrix representing the end-effector position and orientation based on the given joint values and DH parameters.
%
%   Inputs:
%   - numLinks: Number of links in the robotic manipulator
%   - jointAngles: Joint angles of the manipulator (1 x numLinks matrix)
%   - dParams: DH parameter d (1 x numLinks matrix)
%   - aParams: DH parameter a (1 x numLinks matrix)
%   - alphaParams: DH parameter alpha (1 x numLinks matrix)
%   - offsetParams: Offset values for joint angles (1 x numLinks matrix)
%
%   Output:
%   - T: Transformation matrix representing the end-effector position and orientation
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

T = eye(4); % Initialize transformation matrix as identity matrix

for i = 1:numLinks
    cosTheta = cos(jointAngles(i) + offsetParams(i));
    sinTheta = sin(jointAngles(i) + offsetParams(i));
    cosAlpha = cos(alphaParams(i));
    sinAlpha = sin(alphaParams(i));

    T = T * [cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, aParams(i) * cosTheta; ...
             sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, aParams(i) * sinTheta; ...
             0, sinAlpha, cosAlpha, dParams(i); ...
             0, 0, 0, 1];
end

end