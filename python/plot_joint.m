clc; clear; close all;

%% ====== CONFIG ======
joint_file = 'logs_pro_1804/1obs_38_joint.csv';

% Target
target = [-0.407, 0.083, 0.264];

% Obstacle
obs_list = [
    -0.255, 0.073, 0.285
];

axes_list = [
    0.025, 0.025, 0.025
];

colors = lines(6); % 6 joints

%% ====== READ DATA ======
data = readtable(joint_file);

theta = linspace(0, 2*pi, 100);

%% ====== XY VIEW ======
figure; hold on;

for j = 1:6
    x = data.(sprintf('J%d_x', j));
    y = data.(sprintf('J%d_y', j));

    plot(x, y, 'LineWidth', 1.5, 'Color', colors(j,:));
end

% Start point (joint 6)
plot(data.J6_x(1), data.J6_y(1), 'ko', 'MarkerFaceColor','k');

% Target
plot(target(1), target(2), 'g*', 'MarkerSize',10);

% Obstacle
for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    x_obs = obs(1) + axes_len(1)*cos(theta);
    y_obs = obs(2) + axes_len(2)*sin(theta);

    fill(x_obs, y_obs, [0.7 0.7 0.7], ...
        'FaceAlpha',0.4,'EdgeColor','k');
end

xlabel('X'); ylabel('Y');
title('XY - All Joint Trajectories');
axis equal; grid on;

legend('J1','J2','J3','J4','J5','J6','Start','Target');

%% ====== XZ VIEW ======
figure; hold on;

for j = 1:6
    x = data.(sprintf('J%d_x', j));
    z = data.(sprintf('J%d_z', j));

    plot(x, z, 'LineWidth', 1.5, 'Color', colors(j,:));
end

% Start
plot(data.J6_x(1), data.J6_z(1), 'ko', 'MarkerFaceColor','k');

% Target
plot(target(1), target(3), 'g*', 'MarkerSize',10);

% Obstacle
for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    x_obs = obs(1) + axes_len(1)*cos(theta);
    z_obs = obs(3) + axes_len(3)*sin(theta);

    fill(x_obs, z_obs, [0.7 0.7 0.7], ...
        'FaceAlpha',0.4,'EdgeColor','k');
end

xlabel('X'); ylabel('Z');
title('XZ - All Joint Trajectories');
axis equal; grid on;

legend('J1','J2','J3','J4','J5','J6','Start','Target');

%% ====== 3D VIEW ======
figure; hold on;

for j = 1:6
    x = data.(sprintf('J%d_x', j));
    y = data.(sprintf('J%d_y', j));
    z = data.(sprintf('J%d_z', j));

    plot3(x, y, z, 'LineWidth', 1.5, 'Color', colors(j,:));
end

% Start
plot3(data.J6_x(1), data.J6_y(1), data.J6_z(1), ...
    'ko','MarkerFaceColor','k');

% Target
plot3(target(1), target(2), target(3), ...
    'g*','MarkerSize',10);

% Obstacle 3D
[u,v] = meshgrid(linspace(0,2*pi,30), linspace(0,pi,30));

for i = 1:size(obs_list,1)
    obs = obs_list(i,:);
    axes_len = axes_list(i,:);

    xs = obs(1) + axes_len(1)*cos(u).*sin(v);
    ys = obs(2) + axes_len(2)*sin(u).*sin(v);
    zs = obs(3) + axes_len(3)*cos(v);

    surf(xs, ys, zs, ...
        'FaceAlpha',0.3,...
        'EdgeColor','none',...
        'FaceColor',[0.5 0.5 0.5]);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D - All Joint Trajectories');
axis equal; grid on; view(3);

legend('J1','J2','J3','J4','J5','J6','Start','Target');

%% ====== JOINT VELOCITY ======
figure; hold on;

t = data.time;

for j = 1:6
    v = data.(sprintf('qdot%d', j));
    plot(t, v, 'LineWidth',1.5);
end

xlabel('Time (s)');
ylabel('Joint Velocity (rad/s)');
title('Joint Velocities');
grid on;

legend('q1','q2','q3','q4','q5','q6');