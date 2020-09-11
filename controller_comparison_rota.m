%% plot the error of moment of inertia
close all;
Jxx = 0.0347563;
Jyy = 0.0458929;
Jzz = 0.0977;
mass = 1.56779;
sim_t = 50;
bag = rosbag("no_converge.bag");
bag_ICL = rosbag("33-51-91.bag");

%% plot the error of the state

clock_error = select(bag, 'topic', 'clock');
clock_error_msgStructs = readMessages(clock_error, 'DataFormat', 'struct');
time_error = cellfun(@(m) double(m.Clock_.Sec), clock_error_msgStructs);
total_time_error = time_error(end) - time_error(1);
time_ratio_error = sim_t/total_time_error;

clock_error_ICL = select(bag_ICL, 'topic', 'clock');
clock_error_ICL_msgStructs = readMessages(clock_error_ICL, 'DataFormat', 'struct');
time_error_ICL = cellfun(@(m) double(m.Clock_.Sec), clock_error_ICL_msgStructs);
total_time_error_ICL = time_error_ICL(end) - time_error_ICL(1);
time_ratio_error_ICL = sim_t/total_time_error_ICL;

theta_hat_ICL = select(bag_ICL, 'topic', 'theta_hat');
theta_hat_ICL_msgStructs = readMessages(theta_hat_ICL, 'DataFormat', 'struct');
theta_hat_x_ICL = cellfun(@(m) double(m.X), theta_hat_ICL_msgStructs);
theta_hat_y_ICL = cellfun(@(m) double(m.Y), theta_hat_ICL_msgStructs);
theta_hat_z_ICL = cellfun(@(m) double(m.Z), theta_hat_ICL_msgStructs);
theta_hat_x_ICL = theta_hat_x_ICL(1:(int32((length(theta_hat_x_ICL)*time_ratio_error_ICL))));
theta_hat_y_ICL = theta_hat_y_ICL(1:(int32((length(theta_hat_y_ICL)*time_ratio_error_ICL))));
theta_hat_z_ICL = theta_hat_z_ICL(1:(int32((length(theta_hat_z_ICL)*time_ratio_error_ICL))));
t_theta_ICL = linspace(0, sim_t, length(theta_hat_x_ICL));

theta_hat = select(bag, 'topic', 'theta_hat');
theta_hat_msgStructs = readMessages(theta_hat, 'DataFormat', 'struct');
theta_hat_x = cellfun(@(m) double(m.X), theta_hat_msgStructs);
theta_hat_y = cellfun(@(m) double(m.Y), theta_hat_msgStructs);
theta_hat_z = cellfun(@(m) double(m.Z), theta_hat_msgStructs);
theta_hat_x = theta_hat_x(1:(int32((length(theta_hat_x)*time_ratio_error))));
theta_hat_y = theta_hat_y(1:(int32((length(theta_hat_y)*time_ratio_error))));
theta_hat_z = theta_hat_z(1:(int32((length(theta_hat_z)*time_ratio_error))));
t_theta = linspace(0, sim_t, length(theta_hat_x));

figure(1);
subplot(3, 1, 1);
plot(t_theta_ICL, theta_hat_x_ICL, 'LineWidth', 1.5);
hold on;
plot(t_theta, theta_hat_x, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{xx}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jxx, 'm', 'LineWidth', 1.5);
ylim([-0.02, 0.1]);
legend('$\hat{J}_{xx}$ $with$ $ICL$', '$\hat{J}_{xx}$ $without$ $ICL$', '$J_{xx}$', 'Interpreter', 'latex');
title('$Estimated$ $Moment$ $of$ $Inertia$ $w/$ $and$ $w/o$ $ICL$', 'Interpreter', 'latex')

subplot(3, 1, 2);
plot(t_theta_ICL, theta_hat_y_ICL, 'LineWidth', 1.5);
hold on;
plot(t_theta, theta_hat_y, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{yy}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jyy, 'm', 'LineWidth', 1.5);
ylim([-0.05, 0.18]);
legend('$\hat{J}_{yy}$ $with$ $ICL$', '$\hat{J}_{yy}$ $without$ $ICL$', '$J_{yy}$', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t_theta_ICL, theta_hat_z_ICL, 'LineWidth', 1.5);
hold on;
plot(t_theta, theta_hat_z, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{zz}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jzz, 'm', 'LineWidth', 1.5);
ylim([-0.5, 0.5]);
legend('$\hat{J}_{zz}$ $with$ $ICL$', '$\hat{J}_{zz}$ $without$ $ICL$', '$J_{zz}$', 'Interpreter', 'latex');
xlabel('$Time(sec)$', 'Interpreter', 'latex');

figure(2);
subplot(3, 1, 1);
plot(t_theta, theta_hat_x, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{xx}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jxx, 'm', 'LineWidth', 1.5);
ylim([-0.02, 0.1]);
legend('$\hat{J}_{xx}$ $without$ $ICL$', '$J_{xx}$', 'Interpreter', 'latex');
title('$Estimated$ $Moment$ $of$ $Inertia$ $without$ $ICL$', 'Interpreter', 'latex')

subplot(3, 1, 2);
plot(t_theta, theta_hat_y, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{yy}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jyy, 'm', 'LineWidth', 1.5);
ylim([-0.05, 0.18]);
legend('$\hat{J}_{yy}$ $without$ $ICL$', '$J_{yy}$', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t_theta, theta_hat_z, 'LineWidth', 1.5);
y = ylabel('$\hat{J}_{zz}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
yline(Jzz, 'm', 'LineWidth', 1.5);
ylim([-0.5, 0.5]);
legend('$\hat{J}_{zz}$ $without$ $ICL$', '$J_{zz}$', 'Interpreter', 'latex');
xlabel('$Time(sec)$', 'Interpreter', 'latex');