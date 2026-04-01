clc; clear; close all;

%% Section 1: Khởi tạo Robot và Thế giới
folder = fileparts(which('test.m')); 
addpath(genpath(fullfile(folder,'enviroment')));
initialPosition = [0.4; -0.1; 0.25];
myWorld = World(initialPosition);
robot = myWorld.robot.robot;  

myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.45, 0.1, 0.25]); 
attractor = [0.4; 0.4; 0.25];


%% Section 2: IK + Joint limit
theta = myWorld.robot.computeInverseKinematics(initialPosition, robot.homeConfiguration, ...
                false, diag([1, -1, -1]));

[theta_min, theta_max] = getJointLimits(robot);

dt = 0.01;
lambda = 1;

% ===== q init =====
q_sim= - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
[theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);

q_dot_sim = zeros(6,1);

%% ===== OPEN LOOP =====
for k = 1:3000
    x = myWorld.robot.computeForwardKinematics(theta_sim);
    
    qdot_virtual_d = highLevelController(dt, x, attractor, theta_sim, ...
                                            theta_min, theta_max, myWorld);
    
    q_dot_sim = qdot_virtual_d;
    q_sim = q_sim + q_dot_sim * dt;
    
    [theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);
    traj(:,k) = myWorld.robot.computeForwardKinematics(theta_sim);
end

%% ===== CLOSED LOOP =====
f=figure(1);
hold on; grid on; view(0,0);

plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 15);
plot3(traj(1,:), traj(2,:), traj(3,:), 'r--', 'LineWidth', 1);

delta_t = 0.01;
i = 0;

% ===== init q =====
q = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));

[theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
position = myWorld.robot.computeForwardKinematics(theta);

highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
global stopFlag;
stopFlag = false;
while vecnorm(position - attractor) > 0.01 && ~stopFlag 
    
    [t, y_ode] = ode15s (@(t,q) forwardStateDynamic_q(t, q, robot, highLC, myWorld, theta_min, theta_max), ...
        [i * delta_t, (i+1) * delta_t], q);
    
    q = y_ode(end,:)';
    
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    position = myWorld.robot.computeForwardKinematics(theta);
    
    if isvalid(f)
        show(robot, theta, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', true);
        hold on;
        plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10);
        plot3(attractor(1), attractor(2), attractor(3), 'go', 'MarkerSize', 10, 'LineWidth', 2); 
        
        title(['Iteration: ', num2str(i), ' | Dist: ', num2str(vecnorm(position - attractor))]);
        drawnow; 
    end
    
    i = i + 1;
end

%% ===== CONTROLLER =====
function qdot_virtual_d = highLevelController(~, x, attractor, theta, theta_min, theta_max, world)

    %% ===== PARAM =====
    lambda = 1;
    gamma  = 50;%lực kéo end-effector về target
    eta    = 1;% chỉ số hội tụ eta lớn gây hệ rung quỹ đạo gắt, nhỏ thì hệ mượt và chậm
    mu     = 18;% độ gắt khi né obs nếu nhỏ mượt nhưng phản ứng chậm ngược lại lớn thì nhanh và gắt 
    ai     = 160; %độ mạnh của lực đẩy obstacle nếu né mạnh thì quỹ đạo gắt, nhỏ có thể đi xuyên obs nên chọn 2 × gamma
    beta   = 0.85;% để từ 0.7-0.8

    % joint weighting (OPTIONAL - vẫn đúng toán nếu dùng dạng matrix)
    Ki = diag([1 1 1 1 2 5]);

    %% ===== MAPPING =====
    q_virtual = - (1/lambda) * log(max(eps,...
        (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));

    %% ===== ERROR =====
    e = x - attractor;   % (3x1)
    xdot_d = [0;0;0];
    %% ===== JACOBIAN EE =====
    J = geometricJacobian(world.robot.robot, theta, 'tool0');
    Jv = J(4:6,:);   % (3x6)

    % ===== INIT =====
A_task = gamma * e' * Jv;
A_obs_total = zeros(1,6);
B =-gamma*e'*xdot_d;

fprintf('\n=========== DEBUG CONTROLLER ===========\n');
fprintf('||e|| = %.4f\n', norm(e));

%% ===== OBSTACLE =====
if ~isempty(world.listOfObstacles)

    obs = world.listOfObstacles(1);
    x_obs = obs.position;
    x_dot_obs = [0;0;0];

    R1 = obs.ellipseAxes(1);
    R2 = 2 * R1;

    alpha = exp(-(R2^2 - R1^2)) / beta; %beta càn n

    [p, Jp] = linkKinematics(theta);

    for i = 2:6

        if i == 6
            xi  = x;
            Jvi = Jv;
        else
            xi  = p(:,i);
            Jvi = Jp(:,:,i);
        end

        dist_vec = xi - x_obs;
        d_iobs   = dist_vec' * dist_vec;
        dist     = sqrt(d_iobs);

        sigma = 0.1;
        d_i = exp(-(d_iobs - R1^2)/sigma);

        expTerm = exp(-mu*(d_i - alpha));
        aii = ai * mu * expTerm / (1 + expTerm)^2;

        A_obs_i = (aii * d_i * dist_vec' * Jvi) * Ki;

        %% ===== DEBUG LINK =====
        % fprintf('\n--- Link %d ---\n', i);
        % fprintf('dist = %.4f | R1 = %.4f\n', dist, R1);
        % fprintf('d_i = %.4f | aii = %.4f\n', d_i, aii);
        % fprintf('||A_obs_i|| = %.4f\n', norm(A_obs_i));
        % fprintf('d_i - alpha = %.4f\n', d_i - alpha);

        if dist < R1
            fprintf('>>>  COLLISION !!!\n');
        elseif dist < 1.5*R1
            fprintf('>>>  NEAR OBSTACLE\n');
        end

        %% ===== SUM =====
        A_obs_total = A_obs_total + A_obs_i;
        B = B + aii * d_i * dist_vec' * x_dot_obs;
    end
end

%% ===== FINAL A =====
A = A_task - A_obs_total;

% fprintf('\n===== SUMMARY =====\n');
% fprintf('||A_task|| = %.4f\n', norm(A_task));
% fprintf('||A_obs||  = %.4f\n', norm(A_obs_total));
% fprintf('||A_final||= %.4f\n', norm(A));
% 
% %% ===== CHECK DOMINANCE =====
% if norm(A_obs_total) < 0.1 * norm(A_task)
%     fprintf(' Obstacle TOO WEAK → robot IGNORE obstacle\n');
% elseif norm(A_obs_total) > 2 * norm(A_task)
%     fprintf('Obstacle DOMINATES → robot bị đẩy mạnh\n');
% else
%     fprintf('Balance OK\n');
% end
% 
% fprintf('=======================================\n');

    %% ===== PSEUDO-INVERSE =====
    if (A*A') < 1e-6
        Aplus = zeros(6,1);
    else
        Aplus = A' / (A*A' + 1e-4);
    end

    %% ===== LYAPUNOV CONTROL =====
    rhs = -eta * A' - Aplus * B;

    %% ===== MAP BACK =====
    [~, Jg] = jointMapping(q_virtual, theta_min, theta_max, lambda);

    qdot_virtual_d = pinv(Jg) * rhs;
    fprintf('qdot norm: %.3f\n', norm(qdot_virtual_d));
    %% ===== OPTIONAL LIMIT =====
    qdot_virtual_d = max(min(qdot_virtual_d, 5), -5);
end

%% ===== ODE =====
function dqdt = forwardStateDynamic_q(t, q, robot, highLC, world, theta_min, theta_max)

    lambda = 1;

    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);

    x = world.robot.computeForwardKinematics(theta);

    dqdt = highLC(x, theta);

    if norm(dqdt) > 1
        dqdt = dqdt / norm(dqdt) * 1;
    end
end

%% ===== Mapping =====
function [theta, Jg] = jointMapping(q_v, t_min, t_max, lambda)
    theta = (t_max - t_min) ./ (1 + exp(-lambda*q_v)) + t_min;

    dg = lambda*(t_max-t_min).* exp(-lambda*q_v) ./ (1+exp(-lambda*q_v)).^2;
    dg(dg < 1e-9) = 1e-9; 
    Jg = diag(dg);
end

%% ===== Joint limits =====
function [theta_min, theta_max] = getJointLimits(robot)
    n = robot.NumBodies;
    theta_min = []; theta_max = [];
    for i = 1:n
        joint = robot.Bodies{i}.Joint;
        if strcmp(joint.Type,'revolute')
            limits = joint.PositionLimits;
            theta_min = [theta_min; limits(1)];
            theta_max = [theta_max; limits(2)];
        end
    end
end