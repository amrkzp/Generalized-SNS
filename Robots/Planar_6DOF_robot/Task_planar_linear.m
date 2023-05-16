function [p, dp, ddp] = Task_planar_linear(t, T, X_A, X_B)
% Task_planar_linear   Generate a planar linear trajectory between two points.
%
%   [p, dp, ddp] = Task_planar_linear(t, T, X_A, X_B) generates a planar linear trajectory between two points X_A and X_B.
%   The trajectory is parameterized by time t and total duration T.
%
%   Inputs:
%   - t: Time parameter (scalar or array)
%   - T: Total duration of the trajectory (scalar)
%   - X_A: Starting point coordinates (2 x 1 vector)
%   - X_B: Ending point coordinates (2 x 1 vector)
%
%   Outputs:
%   - p: Position vector at time t (2 x 1 vector or 2 x n array)
%   - dp: Velocity vector at time t (2 x 1 vector or 2 x n array)
%   - ddp: Acceleration vector at time t (2 x 1 vector or 2 x n array)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

c = t / T;
xA = X_A(1);
xB = X_B(1);
yA = X_A(2);
yB = X_B(2);

gamma = 6 * c^5 - 15 * c^4 + 10 * c^3;
px = xA + (xB - xA) * gamma;
py = yA + (yB - yA) * gamma;

dpx = T \ (xB - xA) * (30 * c^4 - 60 * c^3 + 30 * c^2);
dpy = T \ (yB - yA) * (30 * c^4 - 60 * c^3 + 30 * c^2);

ddpx = T^2 \ (xB - xA) * (120 * c^3 - 180 * c^2 + 60 * c);
ddpy = T^2 \ (yB - yA) * (120 * c^3 - 180 * c^2 + 60 * c);

p = [px; py];
dp = [dpx; dpy];
ddp = [ddpx; ddpy];

end
