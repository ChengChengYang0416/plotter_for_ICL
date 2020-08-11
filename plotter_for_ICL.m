%% plot the error of moment of inertia
close all;
Jxx = 0.0347563;
Jyy = 0.0458929;
Jzz = 0.0977;
sim_t = 50;
bag = rosbag("geo_big_inertia.bag");

clock = select(bag, 'topic', 'clock');
clock_msgStructs = readMessages(clock, 'DataFormat', 'struct');
time = cellfun(@(m) double(m.Clock_.Sec), clock_msgStructs);
total_time = time(end) - time(1);
time_ratio = sim_t/total_time;

theta_hat = select(bag, 'topic', 'theta_hat');
theta_hat_msgStructs = readMessages(theta_hat, 'DataFormat', 'struct');
theta_hat_x = cellfun(@(m) double(m.X), theta_hat_msgStructs);
theta_hat_y = cellfun(@(m) double(m.Y), theta_hat_msgStructs);
theta_hat_z = cellfun(@(m) double(m.Z), theta_hat_msgStructs);
theta_hat_x = theta_hat_x(1:(int32((length(theta_hat_x)*time_ratio))));
theta_hat_y = theta_hat_y(1:(int32((length(theta_hat_y)*time_ratio))));
theta_hat_z = theta_hat_z(1:(int32((length(theta_hat_z)*time_ratio))));
t_theta = linspace(0, sim_t, length(theta_hat_x));

figure(1);
subplot(3, 1, 1);
plot(t_theta, theta_hat_x, 'LineWidth', 1.5);
y = ylabel('$J_{xx}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.08, 0.41]);
yline(Jxx, 'm', 'LineWidth', 1.5);
ylim([-0.02, 0.1]);
legend('$\hat{J}_{xx}$', '$J_{xx}$', 'Interpreter', 'latex');
title('$Moment$ $of$ $Inertia$ $(kgm^{2}$)', 'Interpreter', 'latex')

subplot(3, 1, 2);
plot(t_theta, theta_hat_y, 'LineWidth', 1.5);
y = ylabel('$J_{yy}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.08, 0.41]);
yline(Jyy, 'm', 'LineWidth', 1.5);
ylim([-0.02, 0.1]);
legend('$\hat{J}_{yy}$', '$J_{yy}$', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t_theta, theta_hat_z, 'LineWidth', 1.5);
y = ylabel('$J_{zz}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.08, 0.41]);
yline(Jzz, 'm', 'LineWidth', 1.5);
ylim([-0.5, 0.5]);
legend('$\hat{J}_{zz}$', '$J_{zz}$', 'Interpreter', 'latex');

theta_error = [theta_hat_x - Jxx, theta_hat_y - Jyy, theta_hat_z - Jzz];
theta = [theta_hat_x, theta_hat_y, theta_hat_z];
theta_error_norm = zeros(length(theta), 1);
for i = 1:length(theta)
    theta_error_norm(i, 1) = norm(theta_error(i, :))/norm(theta(i, :));
end
figure(2)
plot(t_theta, theta_error_norm, 'LineWidth', 1.5);
y = ylabel('$\frac{\left\Vert \widetilde{\theta}\right\Vert }{\left\Vert \theta\right\Vert }$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.115, 0.45]);
set(gca,'Yscale','log');
title('$Estimate$ $errors$', 'Interpreter', 'latex');
ylim([0.001, 310]);
xlabel('$Time(sec)$', 'Interpreter', 'latex');

%% plot the error of the state

clock_error = select(bag, 'topic', 'clock');
clock_error_msgStructs = readMessages(clock_error, 'DataFormat', 'struct');
time_error = cellfun(@(m) double(m.Clock_.Sec), clock_error_msgStructs);
total_time_error = time_error(end) - time_error(1);
time_ratio_error = sim_t/total_time_error;

error = select(bag, 'topic', '/error');
error_msgStructs = readMessages(error, 'DataFormat', 'struct');
%error_msgStructs{1}
error_position_x = cellfun(@(m) double(m.Pose.Pose.Position.X), error_msgStructs);
error_position_y = cellfun(@(m) double(m.Pose.Pose.Position.Y), error_msgStructs);
error_position_z = cellfun(@(m) double(m.Pose.Pose.Position.Z), error_msgStructs);
error_angle_x = cellfun(@(m) double(m.Pose.Pose.Orientation.X), error_msgStructs);
error_angle_y = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), error_msgStructs);
error_angle_z = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), error_msgStructs);
error_psi = cellfun(@(m) double(m.Pose.Pose.Orientation.W), error_msgStructs);
error_velocity_x = cellfun(@(m) double(m.Twist.Twist.Linear.X), error_msgStructs);
error_velocity_y = cellfun(@(m) double(m.Twist.Twist.Linear.Y), error_msgStructs);
error_velocity_z = cellfun(@(m) double(m.Twist.Twist.Linear.Z), error_msgStructs);
error_angu_rate_x = cellfun(@(m) double(m.Twist.Twist.Angular.X), error_msgStructs);
error_angu_rate_y = cellfun(@(m) double(m.Twist.Twist.Angular.Y), error_msgStructs);
error_angu_rate_z = cellfun(@(m) double(m.Twist.Twist.Angular.Z), error_msgStructs);

error_position_x = error_position_x(1:(int32((length(error_position_x)*time_ratio_error))));
error_position_y = error_position_y(1:(int32((length(error_position_y)*time_ratio_error))));
error_position_z = error_position_z(1:(int32((length(error_position_z)*time_ratio_error))));
error_angle_x = error_angle_x(1:(int32((length(error_angle_x)*time_ratio_error))));
error_angle_y = error_angle_y(1:(int32((length(error_angle_y)*time_ratio_error))));
error_angle_z = error_angle_z(1:(int32((length(error_angle_z)*time_ratio_error))));
error_psi = error_psi(1:(int32((length(error_psi)*time_ratio_error))));
error_velocity_x = error_velocity_x(1:(int32((length(error_velocity_x)*time_ratio_error))));
error_velocity_y = error_velocity_y(1:(int32((length(error_velocity_y)*time_ratio_error))));
error_velocity_z = error_velocity_z(1:(int32((length(error_velocity_z)*time_ratio_error))));
error_angu_rate_x = error_angu_rate_x(1:(int32((length(error_angu_rate_x)*time_ratio_error))));
error_angu_rate_y = error_angu_rate_y(1:(int32((length(error_angu_rate_y)*time_ratio_error))));
error_angu_rate_z = error_angu_rate_z(1:(int32((length(error_angu_rate_z)*time_ratio_error))));
error_position_z = error_position_z - 0.04;

t_error = linspace(0, sim_t, length(error_position_x));

figure(3)
subplot(3, 1, 1);
plot(t_error, error_position_x, 'LineWidth', 1.5);
y = ylabel('$e_{p_{x}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
title('$Position$ $Error$ $(m)$', 'Interpreter', 'latex', 'FontSize', 10)
subplot(3, 1, 2);
plot(t_error, error_position_y, 'LineWidth', 1.5);
y = ylabel('$e_{p_{y}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
subplot(3, 1, 3);
plot(t_error, error_position_z, 'LineWidth', 1.5);
y = ylabel('$e_{p_{z}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
xlabel('$Time(sec)$', 'Interpreter', 'latex');

figure(4)
subplot(3, 1, 1);
plot(t_error, error_angle_x, 'LineWidth', 1.5)
y = ylabel('$e_{v_{x}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
title('$Velocity$ $Error$ $(m/s)$', 'Interpreter', 'latex', 'FontSize', 10)
subplot(3, 1, 2);
plot(t_error, error_angle_y, 'LineWidth', 1.5)
y = ylabel('$e_{v_{y}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
subplot(3, 1, 3);
plot(t_error, error_angle_z, 'LineWidth', 1.5)
y = ylabel('$e_{v_{z}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
xlabel('$Time(sec)$', 'Interpreter', 'latex');

figure(5)
subplot(4, 1, 1);
plot(t_error, error_velocity_x, 'LineWidth', 1.5)
y = ylabel('$e_{R_{x}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
title('$Attitude$ $Error$', 'Interpreter', 'latex', 'FontSize', 10)
subplot(4, 1, 2);
plot(t_error, error_velocity_y, 'LineWidth', 1.5)
y = ylabel('$e_{R_{y}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
subplot(4, 1, 3);
plot(t_error, error_velocity_z, 'LineWidth', 1.5)
y = ylabel('$e_{R_{z}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
subplot(4, 1, 4);
plot(t_error, error_psi)
y = ylabel('$\Psi$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
title('$\Psi$', 'Interpreter', 'latex', 'FontSize', 10)
xlabel('$Time(sec)$', 'Interpreter', 'latex');

figure(6)
subplot(3, 1, 1);
plot(t_error, error_angu_rate_x, 'LineWidth', 1.5)
y = ylabel('$e_{\Omega_{x}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
title('$Angular$ $Velocity$ $Error$', 'Interpreter', 'latex', 'FontSize', 10)
subplot(3, 1, 2);
plot(t_error, error_angu_rate_y, 'LineWidth', 1.5)
y = ylabel('$e_{\Omega_{y}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
subplot(3, 1, 3);
plot(t_error, error_angu_rate_z, 'LineWidth', 1.5)
y = ylabel('$e_{\Omega_{z}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.41], 'FontSize', 10);
xlabel('$Time(sec)$', 'Interpreter', 'latex');

%% plot the rotational speed of each rotor

clock_rotor = select(bag, 'topic', 'clock');
clock_rotor_msgStructs = readMessages(clock_rotor, 'DataFormat', 'struct');
time_rotor = cellfun(@(m) double(m.Clock_.Sec), clock_rotor_msgStructs);
total_time_rotor = time_rotor(end) - time_rotor(1);
time_ratio_rotor = sim_t/total_time_rotor;

motor = select(bag, 'topic', '/firefly1/command/motor_speed');
motor_msgStructs = readMessages(motor, 'DataFormat', 'struct');
%motor_msgStructs{1}
motor_speed = cellfun(@(m) double(m.AngularVelocities), motor_msgStructs, 'uniformoutput', false);
motor_speed_arr = zeros(6, length(motor_speed));
for i = 1:length(motor_speed)
    motor_speed_arr(:, i) = motor_speed{i, 1};
end

motor_speed_arr_re = zeros(6, (int32((length(motor_speed_arr(1, :))*time_ratio_rotor))));
for i = 1:6
    motor_speed_arr_re(i, :) = motor_speed_arr(i, 1:(int32((length(motor_speed_arr(1, :))*time_ratio_rotor))));
end

t_rotor = linspace(0, sim_t, length(motor_speed_arr_re(1, :)));
figure(7)
subplot(6, 1, 1);
plot(t_rotor, motor_speed_arr_re(1, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{1}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);
title('$Rotational$ $Speed$ $of$ $Actuators$', 'Interpreter', 'latex', 'FontSize', 10)

subplot(6, 1, 2);
plot(t_rotor, motor_speed_arr_re(2, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{2}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);

subplot(6, 1, 3);
plot(t_rotor, motor_speed_arr_re(3, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{3}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);

subplot(6, 1, 4);
plot(t_rotor, motor_speed_arr_re(4, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{4}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);

subplot(6, 1, 5);
plot(t_rotor, motor_speed_arr_re(5, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{5}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);

subplot(6, 1, 6);
plot(t_rotor, motor_speed_arr_re(6, :), 'LineWidth', 1.5);
y = ylabel('$\Omega_{6}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.1, 0.3], 'FontSize', 10);
xlabel('$Time(sec)$', 'Interpreter', 'latex');