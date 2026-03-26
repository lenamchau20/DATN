clc; clear; close all;

%% ====== CONFIG ======
filenames = {

     'logs2603/2d_1obs_21.csv'
%      'logs2403/1obs_ree_0.01_2.csv'
%      'logs2403/1obs_ree_0.01_3.csv'
%      'logs2403/1obs_rho=5_1.csv'
%      'logs/1obs_ree_5.csv'
};

% Target
target = [-0.475, 0.110, 0.200];

% Obstacle
obs_list = [
 -0.35, 0.100, 0.198;
];

axes_list = [
    %0.025, 0.025, 0.025;
   0.02, 0.02, 0.02;
];

colors = lines(length(filenames)); % màu tự động

%% ====== XY (TOP VIEW) ======
figure; hold on;

for k = 1:length(filenames)
    data = readtable(filenames{k});

    x = data.actual_TCP_pose_0;
    y = data.actual_TCP_pose_1;

    plot(x, y, 'LineWidth', 2, 'Color', colors(k,:));
end

% Start (lấy từ file đầu)
data0 = readtable(filenames{1});
start = [data0.actual_TCP_pose_0(1), data0.actual_TCP_pose_1(1)];

plot(start(1), start(2), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);
plot(target(1), target(2), 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);

% Obstacle
theta = linspace(0, 2*pi, 100);
for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    x_obs = obs(1) + axes_len(1)*cos(theta);
    y_obs = obs(2) + axes_len(2)*sin(theta);

    fill(x_obs, y_obs, [0.7 0.7 0.7], ...
        'FaceAlpha', 0.4, ...
        'EdgeColor', [0.25 0.25 0.25]);
end

xlabel('X (m)');
ylabel('Y (m)');
title('Top View (XY)');
axis equal; grid on;

legend_entries = cell(1, length(filenames)+2);
for k = 1:length(filenames)
    [~, name, ~] = fileparts(filenames{k});
    legend_entries{k} = name;
end
legend_entries{end-1} = 'Start';
legend_entries{end} = 'Target';
legend(legend_entries);

%% ====== XZ (SIDE VIEW) ======
figure; hold on;

for k = 1:length(filenames)
    data = readtable(filenames{k});

    x = data.actual_TCP_pose_0;
    z = data.actual_TCP_pose_2;

    plot(x, z, 'LineWidth', 2, 'Color', colors(k,:));
end

% Start
start = [
    data0.actual_TCP_pose_0(1), 
    data0.actual_TCP_pose_2(1)
];

plot(start(1), start(2), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);
plot(target(1), target(3), 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);

% Obstacle
for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    x_obs = obs(1) + axes_len(1)*cos(theta);
    z_obs = obs(3) + axes_len(3)*sin(theta);

    fill(x_obs, z_obs, [0.7 0.7 0.7], ...
        'FaceAlpha', 0.4, ...
        'EdgeColor', [0.25 0.25 0.25]);
end

xlabel('X (m)');
ylabel('Z (m)');
title('Side View (XZ)');
axis equal; grid on;

legend(legend_entries);

%% ====== 3D TRAJECTORY ======
figure; hold on;

for k = 1:length(filenames)
    data = readtable(filenames{k});

    x = data.actual_TCP_pose_0;
    y = data.actual_TCP_pose_1;
    z = data.actual_TCP_pose_2;

    plot3(x, y, z, 'LineWidth', 2, 'Color', colors(k,:));
end

% Start + Target
start3D = [
    data0.actual_TCP_pose_0(1), 
    data0.actual_TCP_pose_1(1),
    data0.actual_TCP_pose_2(1)
];

plot3(start3D(1), start3D(2), start3D(3), ...
    'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);

plot3(target(1), target(2), target(3), ...
    'g*', 'MarkerSize', 10, 'LineWidth', 1.5);

% Obstacle 3D
[u,v] = meshgrid(linspace(0,2*pi,30), linspace(0,pi,30));

for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    x_surf = obs(1) + axes_len(1)*cos(u).*sin(v);
    y_surf = obs(2) + axes_len(2)*sin(u).*sin(v);
    z_surf = obs(3) + axes_len(3)*cos(v);

    surf(x_surf, y_surf, z_surf, ...
        'FaceAlpha', 0.3, ...
        'EdgeColor', 'none', ...
        'FaceColor', [0.5 0.5 0.5]);
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D TCP Trajectory');

axis equal; grid on;
view(3);

legend(legend_entries);



%%%%%vận tốc

%% ====== VELOCITY ANALYSIS ======
figure; 
subplot(2,1,1); hold on;
for k = 1:length(filenames)
    data = readtable(filenames{k});
    
    % Tính thời gian tương đối bắt đầu từ 0
    t = data.timestamp - data.timestamp(1);
    
    % Lấy các thành phần vận tốc tuyến tính
    vx = data.actual_TCP_speed_0;
    vy = data.actual_TCP_speed_1;
    vz = data.actual_TCP_speed_2;
    
    % Tính độ lớn vận tốc tổng hợp (Norm)
    v_norm = sqrt(vx.^2 + vy.^2 + vz.^2);
    
    plot(t, v_norm, 'LineWidth', 2, 'Color', colors(k,:));
end
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('TCP Speed Magnitude');
legend(legend_entries(1:length(filenames)));

subplot(2,1,2); hold on;
% Vẽ chi tiết các trục của file đầu tiên để phân tích
if ~isempty(filenames)
    data = readtable(filenames{1});
    t = data.timestamp - data.timestamp(1);
    plot(t, data.actual_TCP_speed_0, 'r', 'DisplayName', 'v_x');
    plot(t, data.actual_TCP_speed_1, 'g', 'DisplayName', 'v_y');
    plot(t, data.actual_TCP_speed_2, 'b', 'DisplayName', 'v_z');
end
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Components');
legend show;