%% plot the error of moment of inertia
close all;
Jxx = 0.0347563;
Jyy = 0.0458929;
Jzz = 0.0977;
mass = 1.56779;
sim_t = 50;
bag = rosbag("without_ICL.bag");
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

error = select(bag, 'topic', '/error');
error_msgStructs = readMessages(error, 'DataFormat', 'struct');
%error_msgStructs{1}
error_position_z = cellfun(@(m) double(m.Pose.Pose.Position.Z), error_msgStructs);
error_position_z = error_position_z(1:(int32((length(error_position_z)*time_ratio_error))));
t_error = linspace(0, sim_t, length(error_position_z));

error_ICL = select(bag_ICL, 'topic', '/error');
error_ICL_msgStructs = readMessages(error_ICL, 'DataFormat', 'struct');
%error_msgStructs{1}
error_position_z_ICL = cellfun(@(m) double(m.Pose.Pose.Position.Z), error_ICL_msgStructs);
error_position_z_ICL = error_position_z_ICL(1:(int32((length(error_position_z_ICL)*time_ratio_error_ICL))));

t_error_ICL = linspace(0, sim_t, length(error_position_z_ICL));

figure(1)
plot(t_error_ICL, error_position_z_ICL, 'LineWidth', 1.5);
hold on;
plot(t_error, error_position_z, 'LineWidth', 1.5);
y = ylabel('$e_{x_{3}}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
set(y, 'Units', 'Normalized', 'Position', [-0.11, 0.47]);
legend('$with$ $ICL$', '$without$ $ICL$', 'Interpreter', 'latex');
xlabel('$Time(sec)$', 'Interpreter', 'latex', 'FontSize', 9);
title('$Position$ $Error$ $in$ $z-direction$ $(m)$', 'Interpreter', 'latex', 'FontSize', 9)
ylim([-1.4, 0.6]);