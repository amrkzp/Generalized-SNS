function [dXMin, dXMax] = computeVelocityBounds(X, XMin, XMax, VXMin, VXMax, AXMax, Ts)
% computeVelocityBounds   Compute the velocity bounds for each dimension of the state variable.
%
%   [dXMin, dXMax] = computeVelocityBounds(X, XMin, XMax, VXMin, VXMax, AXMax, Ts) computes the velocity bounds
%   for each dimension of the state variable based on the given position limits, velocity limits, acceleration limits,
%   and sampling time.
%
%   Inputs:
%   - X: Current state variable (nDim x 1 vector)
%   - XMin: Minimum position limits (nDim x 1 vector)
%   - XMax: Maximum position limits (nDim x 1 vector)
%   - VXMin: Minimum velocity limits (nDim x 1 vector)
%   - VXMax: Maximum velocity limits (nDim x 1 vector)
%   - AXMax: Maximum acceleration limits (nDim x 1 vector)
%   - Ts: Sampling time (scalar)
%
%   Outputs:
%   - dXMin: Minimum velocity bounds (nDim x 1 vector)
%   - dXMax: Maximum velocity bounds (nDim x 1 vector)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

tol = 1e-6;
dXMin = max([(XMin - X - tol) / Ts, VXMin, -sqrt(2 * AXMax .* abs(X - XMin + tol))], [], 2);
dXMax = min([(XMax - X + tol) / Ts, VXMax, sqrt(2 * AXMax .* abs(XMax - X + tol))], [], 2);

idx = find(dXMax < dXMin);
for x = 1:numel(idx)
    if dXMax(idx(x)) < dXMin(idx(x)) && dXMax(idx(x)) < VXMin(idx(x))
        dXMax(idx(x)) = dXMin(idx(x));
    end
    if dXMin(idx(x)) > dXMax(idx(x)) && dXMin(idx(x)) > VXMax(idx(x))
        dXMin(idx(x)) = dXMax(idx(x));
    end
end

end
