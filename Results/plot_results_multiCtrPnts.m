close all

% Define colors
Red = [0.8500 0.3250 0.0980];
Yellow = [0.929 0.694 0.125];
Blue = [0 0.4470 0.7410];
Violet = [0.4940 0.1840 0.5560];
Green = [0.4660 0.6740 0.1880];
Cyan = [0.3010 0.7450 0.9330];
Bordeaux = [0.6350 0.0780 0.1840];
Silver = [131, 137, 150] ./ 255;

%% Initialize variables
ee_error = e;
eefile = e * 0;
qfile = rad2deg(Q);
qdotfile = dQ ./ VJntMax';
qddotfile = ddQ ./ AJntMax';

pfile = P_CtrPnts;
pdotfile = V_CtrPnts ./ VCarMax';
pddotfile = A_CtrPnts ./ ACarMax';

dt = 0.001;
t = 0:dt:size(eefile, 1) * dt - dt;

%% Plot
f = figure;
hold on;

% Plot reference lines
yline(PMax(4), '--', 'linewidth', 1.5, 'color', 'red');
yline(PMin(4), '--', 'linewidth', 1.5, 'color', 'red');

% Set initial robot configuration
robot.set_q_and_qdot(Q(1, :), dq1);
plotarm(Q(1, :)', ones(nJnt, 1), 1);
hold on;

k = 1;
for ts = 0:Ts:T
    [pd(k, :), dpd(k, :), ddpd(k, :)] = Task_planar_linear(ts, T, X_A, X_B);
    k = k + 1;
end

% Plot desired trajectory
plot(pd(:, 1), pd(:, 2), '-.', 'LineWidth', 2, 'Color', '#EDB120');
grid on;

% Plot end effector trajectory
ee = T0d;
ee_X = ee(:, 1);
ee_Y = ee(:, 2);
plot(ee_X, ee_Y, 'b', 'LineWidth', 2);

for i = 1:2:size(P_CtrPnts, 2) - 3
    elb_X = P_CtrPnts(:, i);
    elb_Y = P_CtrPnts(:, i + 1);
    plot(elb_X, elb_Y, ':g', 'LineWidth', 2);
end

% Set final robot configuration
robot.set_q_and_qdot(Q(end, :), dq1);
plotarm(Q(end, :)', ones(nJnt, 1), 0.4);

hold off;
xlabel('x [m]', 'Interpreter', 'latex', 'FontSize', 22);
ylabel('y [m]', 'Interpreter', 'latex', 'FontSize', 22);
ylim([-1.5 1.5]);
xlim([0 6]);
grid on;
pbaspect([2 1 1]);
set(gca, 'FontSize', 16);
set(f, 'Position', [500 200 600 400]);

% Add plot title, legends, and adjust font size
% title('Robot Trajectory', 'Interpreter', 'latex', 'FontSize', 24);
% legend('PMax', 'PMin', 'Initial Configuration', 'Desired Trajectory', 'End Effector Trajectory', 'Elbow Trajectories');

% exportgraphics(f,'figures/exp_planner1.pdf','ContentType','vector')

%%

f = figure;

% Set subplot configuration
subplot(2, 1, 1);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Errors [m]', 'Interpreter', 'latex', 'FontSize', 15);
hold on;

% Plot error values
plot(t', ee_error(:, 1), 'LineWidth', 2);
plot(t', ee_error(:, 2), 'LineWidth', 2);

ylim([-0.15 0.05]);
xlim([0.0 t(end)]);
set(gca, 'FontSize', 16);
legend('x', 'y', 'FontSize', 12, 'Interpreter', 'latex', 'location', 'northeast', 'orientation', 'horizontal');
grid on;

subplot(2, 1, 2);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Scaling factor', 'Interpreter', 'latex', 'FontSize', 15);
hold on;

% Plot scaling factor
plot(t', scaleFactor, '-k', 'LineWidth', 2);

xlim([0.0 t(end)]);
ylim([0.0 1.05]);
set(gca, 'FontSize', 16);
grid on;

set(f, 'Position', [500 200 800 500]);

% exportgraphics(f,'figures/exp_planner2.pdf','ContentType','vector')

%%

f=figure;
% figure_configuration_IEEE_standard
hold on
xlabel('Time [s]','Interpreter','latex', 'FontSize', 15)
ylabel('Joint positions [deg]','Interpreter','latex', 'FontSize', 15)
plot(t',qfile(:,1),'LineWidth',2);
plot(t',qfile(:,2),'LineWidth',2);
plot(t',qfile(:,3),'LineWidth',2);
plot(t',qfile(:,4),'LineWidth',2);
plot(t',qfile(:,5),'LineWidth',2);
plot(t',qfile(:,6),'LineWidth',2);
yline(90,'--k','linewidth',1.5);
yline(-90,'--k','linewidth',1.5);
legend('${q}_1$','${q}_2$','${q}_3$','${q}_4$','${q}_5$','${q}_6$','Orientation','horizontal','location','northoutside');
set(legend,'FontSize',16,'Interpreter','latex');
xlim([0.0 t(end)])
ylim([-120 120])
set(gca,'FontSize',16);
set(f,'Position',[500 200 800 300])
grid on
% exportgraphics(f,'figures/exp_planner3.pdf','ContentType','vector')


f=figure; 
% figure_configuration_IEEE_standard
xlabel('Time [s]','Interpreter','latex', 'FontSize', 15)
ylabel('Joint velocities','Interpreter','latex', 'FontSize', 15)
hold on
plot(t',qdotfile(:,1),'LineWidth',2);
plot(t',qdotfile(:,2),'LineWidth',2);
plot(t',qdotfile(:,3),'LineWidth',2);
plot(t',qdotfile(:,4),'LineWidth',2);
plot(t',qdotfile(:,5),'LineWidth',2);
plot(t',qdotfile(:,6),'LineWidth',2);
hold on
yline(-1,'--','linewidth',1.5,'color',Silver);
yline(1,'--','linewidth',1.5,'color',Silver);
% patch(x_shade,y_shade,'red','facealpha',0.1,'edgecolor','none')
legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','Orientation','horizontal','location','northoutside');
set(legend,'FontSize',16,'Interpreter','latex');
xlim([0.0 t(end)])
ylim([-1.1 1.1])
set(gca,'FontSize',16);
set(f,'Position',[500 200 800 300])
grid on
% exportgraphics(f,'figures/exp_planner4.pdf','ContentType','vector')

f=figure;
% figure_configuration_IEEE_standard
hold on
ylabel('Control point positions [m]','Interpreter','latex', 'FontSize', 22);
xlabel('Time [s]','Interpreter','latex', 'FontSize', 16);
for i=2:2:size(pddotfile,2)-2
    plot(t(1:length(pfile)),pfile(:,i),'linewidth',2),hold on
end
yline(cartPosLimYMax ,'--k','linewidth',1.5)
yline(cartPosLimYMin ,'--k','linewidth',1.5)
grid on;
xlim([0.0 t(end)])
ylim([-1.1 1.1]);
legend('$p_{ctr,1}$','$p_{ctr,2}$','$p_{ctr,3}$','$p_{ctr,4}$','$p_{ctr,5}$','Orientation','horizontal','location','northoutside');
set(legend,'FontSize',16,'Interpreter','latex');
set(gca,'FontSize',16);
set(f,'Position',[500 200 800 300])
grid on
% exportgraphics(f,'figures/exp_planner5.pdf','ContentType','vector')


f=figure;
% figure_configuration_IEEE_standard
hold on
xlabel('Time [s]','Interpreter','latex', 'FontSize', 15)
ylabel('Control point velocities','Interpreter','latex', 'FontSize', 22);
for i=2:2:size(pdotfile,2)-2
    plot(t(1:length(pdotfile)),pdotfile(:,i),'linewidth',2),hold on
end
yline(1,'--k','linewidth',1.5)
yline(-1,'--k','linewidth',1.5)
grid on;
xlim([0.0 t(end)])
ylim([-1.1 1.1]);
legend('$\dot{p}_{ctr,1}$','$\dot{p}_{ctr,2}$','$\dot{p}_{ctr,3}$','$\dot{p}_{ctr,4}$','$\dot{p}_{ctr,5}$','Orientation','horizontal','location','northoutside');
set(legend,'FontSize',16,'Interpreter','latex');
set(gca,'FontSize',16);
set(f,'Position',[500 200 800 300])
grid on
% exportgraphics(f,'figures/exp_planner6.pdf','ContentType','vector')

% Plotting execution times
f=figure;
plot(t',smooth(exc_time_GSNS,25),'linewidth',2)
hold on
plot(t',smooth(exc_time_FGSNS,25),'Color',Green,'linewidth',2)
plot(t',smooth(exc_time_QP,25),'Color',Red,'linewidth',2)
hold off
xlabel('Time [s]','Interpreter','latex', 'FontSize', 15)
ylabel('Execution time [s]','Interpreter','latex', 'FontSize', 22);
ylim([0 5e-3])
xlim([0 t(end)])
legend('GSNS','FGSNS','QP')
grid on
set(legend,'FontSize',16,'Interpreter','latex');
set(gca,'FontSize',16);
set(f,'Position',[500 200 800 300])
% title('Execution Time Analysis', 'Interpreter', 'latex', 'FontSize', 24);
