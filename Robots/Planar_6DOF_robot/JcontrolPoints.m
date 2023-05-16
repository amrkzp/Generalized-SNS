function  J_ctrPnts = JcontrolPoints(q1,q2,q3,q4,q5,q6)
% computeControlPointsJacobian   Compute the Jacobians of the intermediate control points.
%
%   J_ctrPnts = computeControlPointsJacobian(q1, q2, q3, q4, q5, q6) computes the Jacobians of the intermediate control points
%   based on the joint angles q1, q2, q3, q4, q5, q6.
%
%   Inputs:
%   - q1, q2, q3, q4, q5, q6: Joint angles of the robotic manipulator (scalar)
%
%   Output:
%   - J_ctrPnts: Jacobians of the intermediate control points (2n x n matrix)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

J_ctrPnts(:,:,1) = [-sin(q1), 0, 0, 0, 0, 0;
    cos(q1), 0, 0, 0, 0, 0];


J_ctrPnts(:,:,2) =[- sin(q1 + q2) - sin(q1), -sin(q1 + q2), 0, 0, 0, 0;
    cos(q1 + q2) + cos(q1),  cos(q1 + q2), 0, 0, 0, 0];


J_ctrPnts(:,:,3) =[- sin(q1 + q2 + q3) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2), -sin(q1 + q2 + q3), 0, 0, 0;
    cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3) + cos(q1 + q2),  cos(q1 + q2 + q3), 0, 0, 0];


J_ctrPnts(:,:,4) =[- sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2), - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4), -sin(q1 + q2 + q3 + q4), 0, 0;
    cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4),  cos(q1 + q2 + q3 + q4), 0, 0];


J_ctrPnts(:,:,5) =[- sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3 + q4), -sin(q1 + q2 + q3 + q4 + q5), 0;
    cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),  cos(q1 + q2 + q3 + q4 + q5), 0];


J_ctrPnts(:,:,6) =[- sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3 + q4 + q5 + q6), -sin(q1 + q2 + q3 + q4 + q5 + q6);
    cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5),  cos(q1 + q2 + q3 + q4 + q5 + q6)];

end