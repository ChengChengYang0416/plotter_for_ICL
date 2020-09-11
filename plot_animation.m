%% plot the error of moment of inertia
close all;
Jxx = 0.0347563;
Jyy = 0.0458929;
Jzz = 0.0977;
mass = 1.56779;
sim_t = 20;
bag = rosbag("33-51-91.bag");

clock = select(bag, 'topic', 'clock');
clock_msgStructs = readMessages(clock, 'DataFormat', 'struct');
time = cellfun(@(m) double(m.Clock_.Sec), clock_msgStructs);
total_time = time(end) - time(1);
time_ratio = sim_t/total_time;

theta_hat = select(bag, 'topic', 'theta_hat');
theta_m_hat = select(bag, 'topic', 'theta_m_hat');
theta_hat_msgStructs = readMessages(theta_hat, 'DataFormat', 'struct');
theta_m_hat_msgStructs = readMessages(theta_m_hat, 'DataFormat', 'struct');
theta_hat_x = cellfun(@(m) double(m.X), theta_hat_msgStructs);
theta_hat_y = cellfun(@(m) double(m.Y), theta_hat_msgStructs);
theta_hat_z = cellfun(@(m) double(m.Z), theta_hat_msgStructs);
theta_m_hat_R = cellfun(@(m) double(m.Z), theta_m_hat_msgStructs);
theta_hat_x = theta_hat_x(1:(int32((length(theta_hat_x)*time_ratio))));
theta_hat_y = theta_hat_y(1:(int32((length(theta_hat_y)*time_ratio))));
theta_hat_z = theta_hat_z(1:(int32((length(theta_hat_z)*time_ratio))));
theta_m_hat_R = theta_m_hat_R(1:(int32((length(theta_m_hat_R)*time_ratio))));
t_theta = linspace(0, sim_t, length(theta_hat_x));
t_theta_m = linspace(0, sim_t, length(theta_m_hat_R));

figure(1)
subplot(4, 1, 1);
yline(Jxx, 'm', 'LineWidth', 1.5);
hold on;
subplot(4, 1, 2);
yline(Jyy, 'm', 'LineWidth', 1.5);
hold on;
subplot(4, 1, 3);
yline(Jzz, 'm', 'LineWidth', 1.5);
hold on;
subplot(4, 1, 4);
yline(mass, 'm', 'LineWidth', 1.5);
hold on;

% Initialize video
myVideo = VideoWriter('myVideoFile_20');
myVideo.FrameRate = 100;
open(myVideo)

for i = 1:length(t_theta)
    subplot(4, 1, 1);
    plot(t_theta(i), theta_hat_x(i), '.b');
    hold on;
%     plot(t_theta(i), Jxx, '.m');
%     hold on;
    y = ylabel('$\hat{J}_{xx}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    ylim([-0.02, 0.1]);
    xlim([0, sim_t]);
    %legend('$\hat{J}_{xx}$', '$J_{xx}$', 'Interpreter', 'latex');
    title('$Estimated$ $Moment$ $of$ $Inertia$ $and$ $Mass$', 'Interpreter', 'latex')

    subplot(4, 1, 2);
    plot(t_theta(i), theta_hat_y(i), '.b');
    hold on;
%     plot(t_theta(i), Jyy, '.m');
%     hold on;
    y = ylabel('$\hat{J}_{yy}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    %yline(Jyy, 'm', 'LineWidth', 1.5);
    ylim([-0.05, 0.18]);
    xlim([0, sim_t]);
    %legend('$\hat{J}_{yy}$', '$J_{yy}$', 'Interpreter', 'latex');

    subplot(4, 1, 3);
    plot(t_theta(i), theta_hat_z(i), '.b');
    hold on;
%     plot(t_theta(i), Jzz, '.m');
%     hold on;
    y = ylabel('$\hat{J}_{zz}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    %yline(Jzz, 'm', 'LineWidth', 1.5);
    ylim([-0.5, 0.5]);
    xlim([0, sim_t]);
    %legend('$\hat{J}_{zz}$', '$J_{zz}$', 'Interpreter', 'latex');

    subplot(4, 1, 4);
    plot(t_theta(i), theta_m_hat_R(i), '.b');
    hold on;
%     plot(t_theta(i), mass, '.m');
%     hold on;
    y = ylabel('$\hat{\theta}_{m}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.095, 0.46]);
    %yline(mass, 'm', 'LineWidth', 1.5);
    ylim([0, 3]);
    xlim([0, sim_t]);
    %legend('$\hat{\theta}_{m}$', '$\theta_{m}$', 'Interpreter', 'latex');
    xlabel('$Time(sec)$', 'Interpreter', 'latex');

    pause(0.001)
    frame = getframe(gcf);
    writeVideo(myVideo, frame);
end
close(myVideo)

% theta_error = [theta_hat_x - Jxx, theta_hat_y - Jyy, theta_hat_z - Jzz];
% theta = [theta_hat_x, theta_hat_y, theta_hat_z];
% theta_error_norm = zeros(length(theta), 1);
% for i = 1:length(theta)
%     theta_error_norm(i, 1) = norm(theta_error(i, :))/norm(theta(i, :));
% end