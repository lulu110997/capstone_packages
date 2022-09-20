clear
close all
clc

use_mp_pinv = 0;
              
% Create a 5 DoF 2D manipulator
P1 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-2, 2], 'offset', 0); % PRISMATIC Link along y
P2 = Link('theta', -pi/2, 'a',0 , 'alpha', -pi/2, 'qlim', [-2, 2], 'offset', 0); % PRISMATIC Link along x

L1 = Link('d',0,'a', 1.0, 'alpha', 0, 'offset', 0);
L2 = Link('d',0,'a', 1.3, 'alpha', 0);
L3 = Link('d',0,'a', 0.7, 'alpha', 0);
R = SerialLink([P1 P2 L1 L2 L3],'name','redundant planar robot');
q_guess = [0 0 0.7791   -1.8074   -0.1910];

R.base = trotx(-pi/2);
% R.jacob0([0 0 0.8917    0.0000   -0.5356])
% R.teach(q_guess, 'scale', 0.8)
%    -2.0007   -0.9097   -0.2030
%    -2.2875   -1.9933   -1.1116
%          0         0         0
%          0         0         0
%          0         0         0
%     1.0000    1.0000    1.0000

% Simulation parameters
t = 1;             % Total time (s)
deltaT = 1/500;      % Corresponds with the 500Hz control frequency
steps = t/deltaT;   % No. of steps for simulation
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
trajectory = zeros(3,steps);             % Array for x-y-z trajectory

waypoints = [ [1 1.5]; ... %C1
              [-1 1.5]; ... %C2
              [-1 2.5]; ... %C3
              [1 2.5]; ... %C4
              [1 1.5]; ... % Back to start
              [-3.5 3.5]; ... %WP1
              [-3 3]; ... %C5
              [-1 3]; ... %C6
              [-1 2]; ... %C7
              [-3 2]; ... %C8
              [-3 3] ... % Back to start
              ];
dist_per_point = (2.793 - pi/9)/length(trajectory);
negative_rot = [4 7 11];
no_rot = [3 5 8 10];
qDots = [];
qMatrix = [];
m = [];
trajectories = [];

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    trajectory(1,i) = (1-s(i))*1 + s(i)*-1; % Points in x
    trajectory(2,i) = 1.5; % Points in y
    theta(3,i) = pi/9  + i*dist_per_point; % Yaw angle
end
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) trajectory(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
last_q = R.ikunc(T, q_guess); % Solve joint angles to achieve first waypoint
last_yaw = pi/9;

for wp_idx = 2:length(waypoints)
    % Determine direction of rotation
    if ismember(wp_idx, no_rot)
            dist_per_point = 0;
    elseif ismember(wp_idx, negative_rot)
        dist_per_point = -(2.793 - pi/9)/length(trajectory);
    else
        dist_per_point = (2.793 - pi/9)/length(trajectory);
    end
    % Generate the trajectory
    for i=1:length(trajectory)
        trajectory(1,i) = (1-s(i))*T(1,end) + s(i)*waypoints(wp_idx); % Points in x
        trajectory(2,i) =(1-s(i))*T(2,end) + s(i)*waypoints(11+wp_idx); % Points in y
        theta(3,i) = last_yaw + i*dist_per_point;                 % Yaw angle
    end
    if use_mp_pinv
        [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
    else
        [new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
    end
    qDots = [qDots; qDot];
    qMatrix = [qMatrix; new_qMatrix];
    m = [m; new_m];
    trajectories = [trajectories trajectory];
    % Obtain current EE pose
    last_q = qMatrix(end,:);
    T = R.fkine(last_q).T;
    rpy = tr2rpy(T);
    last_yaw = rpy(3);
end

% Plot the results
close all
fps = 10000;
figure('Position', get(0, 'Screensize'))
final_cartesian_xy_points = trajectories;
check_traj(final_cartesian_xy_points)

% R.plot(qMatrix(1:2:5000,:),'trail','r-', 'fps', fps, 'workspace', [-4, 4 -4 4 -4 4], 'zoom', 1.8, 'movie', 'frames', 'view', [0 90])

for i=1:10:5000
    R.plot(qMatrix(i, 1:5),'trail','r-', 'fps', fps, 'workspace', [-4, 4 -4 4 -4 4], 'zoom', 1.8)
end

% R.plot(qMatrix(end, 1:5),'trail','r-', 'fps', fps, 'workspace', [-4, 4 -4 4 -4 4], 'zoom', 1.8)

%
figure('Position', get(0, 'Screensize')) % x,y position of EE and manipulability
subplot(2,1,1)
hold on
for i=1:500:5000
    if i <= 2000
        plot(final_cartesian_xy_points(1, i:i+499), final_cartesian_xy_points(2,i:i+499), 'r-')
    elseif i>=3000
        plot(final_cartesian_xy_points(1, i:i+499), final_cartesian_xy_points(2,i:i+499), 'g-')
    else
        plot(final_cartesian_xy_points(1, i:i+499), final_cartesian_xy_points(2,i:i+499), 'b-')
    end
end
title('EE position ')
ylabel('y pos')
xlabel('x pos')
hold off

subplot(2,1,2)
hold on
for i=1:499:4990
    if i <= 1500
        plot(i:i+498, m(i:i+498,:), 'r-')
    elseif i>=2500
        plot(i:i+498, m(i:i+498,:), 'g-')
    else
        plot(i:i+498, m(i:i+498,:), 'b-')
    end
end
for i=1:499:4500
    xline(i+1, 'LineWidth',2, 'HandleVisibility','off')
end
title('Manipulability')
ylim([0 2])
hold off

figure('Position', get(0, 'Screensize')) % joint velocity and manipulability
subplot(3,1,1)
hold on
plot(qDots(:, 1), 'r-')
plot(qDots(:, 2), 'g-')
for i=1:499:4500
    xline(i+1, 'LineWidth',2, 'HandleVisibility','off')
end
hold off
legend('Joint1','Joint2')
axis padded
title('Prismatic Joint Velocities')
ylabel('Joint velocity')
xlabel('Step')
ylim([-6 6])

subplot(3,1,2)
hold on
plot(qDots(:, 3), 'b-')
plot(qDots(:, 4), 'k-')
plot(qDots(:, 5), 'm-')
for i=1:499:4500
    xline(i+1, 'LineWidth',2, 'HandleVisibility','off')
end
hold off
legend('Joint3', 'Joint4', 'Joint5')
axis padded
title('Revolute Joint Velocities ')
ylabel('Joint velocity')
xlabel('Step')
ylim([-8 8])

subplot(3,1,3)
hold on
for i=1:499:4990
    if i <= 1500
        plot(i:i+498, m(i:i+498,:), 'r-')
    elseif i>=2500
        plot(i:i+498, m(i:i+498,:), 'g-')
    else
        plot(i:i+498, m(i:i+498,:), 'b-')
    end
end
for i=1:499:4500
    xline(i+1, 'LineWidth',2, 'HandleVisibility','off')
end
title('Manipulability')
hold off
ylim([0 2])

figure('Position', get(0, 'Screensize'))
subplot(5,1,1)
scatter(m, abs(qDots(:,1)))
subplot(5,1,2)
scatter(m, abs(qDots(:,2)))
subplot(5,1,3)
scatter(m, abs(qDots(:,3)))
subplot(5,1,4)
scatter(m, abs(qDots(:,4)))
subplot(5,1,5)
scatter(m, abs(qDots(:,5)))

if use_mp_pinv
    save('params.mat', 'm', 'trajectories', 'qDots', 'qMatrix')
else
    save('params.mat', 'beta', 'm', 'min_allowable_weight', 'tau', 'trajectories', 'SCALE_NULL_VEL', 'qDots', 'qMatrix')
end
saveas(figure(2), "ee_pos.jpg")
saveas(figure(3), "j_vels.jpg")
saveas(figure(4), "scatter plot.jpg")

for i=1:3
    pause(0.3)
    beep
end
%%

function jac_pinv_weighted = pinv_weighted(weight, jacobian)

        weights_sqrt = power(weight, -0.5);
        weights_matrix_sqrt = diag(weights_sqrt);

        j_aux = jacobian*weights_matrix_sqrt;
        jac_pinv_weighted = weights_matrix_sqrt*pinv(j_aux);

end

function [qMatrix, m, qdot] = RMRC_with_optim(trajectory, theta, deltaT, q0, R)

    % Track the trajectory with RMRC
    qMatrix = zeros(length(trajectory), R.n);
    m = zeros(length(trajectory)-1,1);             % Array for Measure of Manipulability
    qdot = zeros(length(trajectory)-1, R.n);          % Array for joint velocities
    
    qMatrix(1,:) = q0; % Initialise first waypoint

    % Numerical calculation for Jacobian derivative wrt joint pos    
    h = 10^-4;
    delta_matrix = diag(h+zeros(R.n,1));
    numerical_dm_dq_ = nan(1,R.n); % Init Jacobian deriv matrix
    
    SCALE_NULL_VEL = 16;
    min_allowable_weight = power(10,-5);
    beta = 0.77;
    tau = 0.1;
    primary_weight = zeros(1,R.n);
    secondary_weight = zeros(1,R.n);
    assignin('base','SCALE_NULL_VEL',SCALE_NULL_VEL)
    assignin('base','min_allowable_weight',min_allowable_weight)
    assignin('base','beta',beta)
    assignin('base','tau',tau)
    
    for i = 1:length(trajectory)-1
        % Obtain the current joint config
        curr_q = qMatrix(i,:);
        
        T = R.fkine(curr_q);                                           % Get forward transformation at current joint state
        T = T.T;
        deltaX = trajectory(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        linear_velocity = linear_velocity(1:2);
        angular_velocity = angular_velocity(3);
        xdot = [linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        
        % Obtain the current manipulability, 2D Jacobian for the 
        % current joint config and its pseudoinverse
        J = R.jacob0(curr_q);
        J = [J(1:2, :); J(6, :)];
        j_man = J(:,3:end);
        m(i) = sqrt(det(j_man*j_man'));
        
        % The arm is utilised depending on the manipulability
        beta_power = (m(i) + min_allowable_weight) / tau;
        arm_weight = power(beta, beta_power);
        if arm_weight > 1
            error('arm weight greater than 1!')
        end
        platform_weight = 1.0 - arm_weight;

        for j=1:R.n
            if j == 1 || j == 2
                primary_weight(1,j) = platform_weight;
                if arm_weight < 0.4
                    secondary_weight(1,j) = arm_weight; % should be 1. Was arm_weight weight. Can also try platform weight
                else
                    secondary_weight(1,j) = 1; % should be 1. Was arm_weight weight. Can also try platform weight
                end
            else
                primary_weight(1,j) = arm_weight;
                if arm_weight < 0.4
                    secondary_weight(1,j) = arm_weight; % should be 1. Was arm_weight weight. Can also try platform weight
                else
                    secondary_weight(1,j) = min_allowable_weight; % should be min allow. Was platform_weight
                end
            end
        end
        
        
        pinvJ_primary = pinv_weighted(primary_weight, J);
        pinvJ_secondary = pinv_weighted(secondary_weight ,J);
        
%         pinvJ_primary = pinv(J);
%         pinvJ_secondary = pinv(J);
        % End code for weight selection
        
        
        for k = 1:R.n % joints
            
            % Find the 'next; and 'previous' Jacobian given a movement in
            % the independent (joint) variable and numerically calculate
            % the jacobian wrt the independent variable
            jac_p = R.jacob0(curr_q + delta_matrix(k,:));
            jac_p = [jac_p(1:2,:); jac_p(6,:)];
            jac_m = R.jacob0(curr_q - delta_matrix(k,:));
            jac_m = [jac_m(1:2,:); jac_m(6,:)];
            partialJ_partialqk = (jac_p - jac_m)/(2*h);     
            
            numerical_dm_dq_(k) = m(i)*trace(partialJ_partialqk*pinv(J));
        
        end

        
        % Update next joint state based on joint velocities
%         (eye(R.n)-pinvJ*J)*numerical_dm_dq_'
%         (pinvJ*xdot)
        qdot(i,:) = (pinvJ_primary*xdot) + SCALE_NULL_VEL*(eye(R.n)-pinvJ_secondary*J)*numerical_dm_dq_';
        qMatrix(i+1,:) = curr_q + deltaT*qdot(i,:);

    end
    % Manipulabilityu at final state
%     J = R.jacob0(qMatrix(end,:));
%     J = [J(1:2, :); J(6, :)];
%     j_man = J(:,3:end);
%     m(end) = sqrt(det(j_man*j_man'));

end

function [qMatrix, m, qdot] = RMRC_traj_tracking(trajectory, theta, deltaT, q0, R)
    % Track the trajectory with RMRC
    qMatrix = zeros(length(trajectory), R.n);
    m = zeros(length(trajectory)-1,1);             % Array for Measure of Manipulability
    qdot = zeros(length(trajectory)-1, R.n);          % Array for joint velocities
    
    qMatrix(1,:) = q0; % Initialise first waypoint
    
    for i = 1:length(trajectory)-1
        T = R.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        T = T.T;
        deltaX = trajectory(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        linear_velocity = linear_velocity(1:2);
        angular_velocity = angular_velocity(3);
        xdot = [linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = R.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        J = [J(1:2, :); J(6, :)];
        j_man = J(:,3:end);
        m(i) = sqrt(det(j_man*j_man'));
        invJ = pinv(J);                                   % MP Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the vector)

        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities

    end
    % Manipulability at final state
%     J = R.jacob0(qMatrix(end,:));
%     J = [J(1:2, :); J(6, :)];
%     j_man = J(:,3:end);
%     m(end) = sqrt(det(j_man*j_man'));
end

function check_traj(trajectory)
% Check trajectory
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'k.')

end