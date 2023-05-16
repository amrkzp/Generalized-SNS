function plotarm(q, L, Alpha)
% plotArm   Plot the robotic arm configuration.
%
%   plotArm(q, L, alpha) plots the configuration of the robotic arm with joint angles q and link lengths L.
%   The transparency of the arm links is set using the alpha parameter.
%
%   Inputs:
%   - q: Joint angles of the robotic arm (nJoints x 1 vector)
%   - L: Link lengths of the robotic arm (nJoints x 1 vector)
%   - alpha: Transparency of the arm links (scalar)
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

[~, p, ~, ~] = Robot_Compute(numel(q), q, L, 1e-3);
p = cat(2, [0; 0], p);
hold on;

for i = 1:size(p, 2)-1
    hline = plot([p(1, i), p(1, i+1)], [p(2, i), p(2, i+1)], 'Color', 'k', 'LineWidth', 3);
    hline.Color = [hline.Color Alpha];  
    hold on;
end

hline = scatter(p(1, :), p(2, :), 'r', 'MarkerFaceColor', 'r', 'LineWidth', 3);
alpha(hline, Alpha);

grid on;
end
