clear
close
clc

not_optimise_rmrc = 1;

% Create a 5 DoF 2D manipulator
scale_lengths = 1.25;
P1 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-2, 2], 'offset', 0); % PRISMATIC Link along y
P2 = Link('theta', -pi/2, 'a',0 , 'alpha', -pi/2, 'qlim', [-2, 2], 'offset', 0); % PRISMATIC Link along x

% L1 = Link('d',0,'a',scale_lengths*0.7127,'alpha', 0, 'offset', pi); %0.6127 +0.1
% L2 = Link('d',0,'a',scale_lengths*0.7723,'alpha', 0); %0.5723 +0.2
% L3 = Link('d',0,'a',scale_lengths*0.7,'alpha', 0); %.0.4 +0.3
% L4 = Link('d',0,'a',scale_lengths*0.6783,'alpha', 0); %0.5283 +0.15
% R = SerialLink([P1 P2 L1 L2 L3 L4],'name','redundant planar robot');
% q_guess = [0 0 0.6833+1.57    0.2385    0.0073   -0.1445];

L1 = Link('d',0,'a',scale_lengths*0.9127,'alpha', 0, 'offset', pi); %0.6127 +0.1
L2 = Link('d',0,'a',scale_lengths*0.9723,'alpha', 0); %0.5723 +0.2
L3 = Link('d',0,'a',scale_lengths*0.9,'alpha', 0); %.0.4 +0.3
R = SerialLink([P1 P2 L1 L2 L3],'name','redundant planar robot');
q_guess = [0 0 0.6833+1.57    0.2385   -0.1445];

% L1 = Link('d',0,'a',scale_lengths*0.6127,'alpha', 0, 'offset', pi);
% L2 = Link('d',0,'a',scale_lengths*0.5723,'alpha', 0);
% L3 = Link('d',0,'a',scale_lengths*0.4,'alpha', 0);
% L4 = Link('d',0,'a',scale_lengths*0.5283,'alpha', 0);
% L5 = Link('d',0,'a',scale_lengths*0.6,'alpha', 0);
% R = SerialLink([P1 P2 L1 L2 L3 L4 L5],'name','redundant planar robot');
% q_guess = [0 0 0.6833+1.57    0.2385    0.0073   -0.1445   -0.4353];

% display(sum(R.a))

R.base = trotx(-pi/2);

% R.jacob0([0 0 0.2000    0.4398    0.3770    1.2566    0.4398])
% R.teach([0.2, 0.4398    0.3770    1.2566    0.4398], 'scale', 0.8)
%     0.4786   -0.2720   -0.8459   -1.1090   -0.6822
%     1.8200    1.6678    1.2407    0.8155    0.3116
%          0         0         0         0         0
%          0         0         0         0         0
%          0         0         0         0         0
%     1.0000    1.0000    1.0000    1.0000    1.0000

% Simulation parameters
t = 2;             % Total time (s)
deltaT = 0.002;      % Corresponds with the 500Hz control frequency
steps = t/deltaT;   % No. of steps for simulation
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
trajectory = zeros(3,steps);             % Array for x-y-z trajectory

% Create the sinusoidal trajectory
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    trajectory(1,i) = (1-s(i))*2.8 + s(i)*1.5; % Points in x
    trajectory(2,i) = 1.6 + 0.13*sin(i*0.013); % Points in y
    theta(3,i) = pi/9;                 % Yaw angle
end
[trajectory(1,:), trajectory(2,:)] = rotate_points_theta_rad(trajectory(1,:), trajectory(2,:), pi/3);

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) trajectory(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
q0 = R.ikunc(T, q_guess); % Solve joint angles to achieve first waypoint

if not_optimise_rmrc
    [qMatrix, m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, q0, R);
else
    [qMatrix, m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, q0, R);
end
qDots = qDot;

% Generate a trajectory using by creating a polynomial from a set of points
% Obtain current EE pose
last_q = qMatrix(end,:);
T = R.fkine(last_q).T;
rpy = tr2rpy(T);
last_yaw = rpy(3);
% Create a set of points
% x_points = [T(1,end), 1.68, 1.22, 0.85, 0.68 0.4, 0.36, 0.3, 0.28, 0, -0.1, -0.17];
y_points = [T(2,end), 1.1, 1.35, 1.4, 1.47, 1.5, 1.8, 2.5, 2.8, 2.879, 3, 3.08];
x_points = lspb(T(1,end), -0.05, length(y_points))';
% y_points = lspb(T(2,end), 3, 5)';
% interpolated_points = fnplt(cscvn([x_points; y_points]));
% Obtain the polynomial
p = polyfit(x_points, y_points, length(y_points)-1);
x1 = linspace(T(1,end), 0, steps);
f1 = polyval(p, x1);
if abs(f1(1) - T(2,end)) > 0.01
    error('polynomial bad')
end
% Generate the trajectory
trajectory = [x1; f1; zeros(1,steps)];
dist_per_point = ((0.7*pi)/2 - last_yaw)/length(trajectory);
for i=1:length(trajectory)
    theta(:,i) = [0 0 (last_yaw + i*dist_per_point)]';
end
if not_optimise_rmrc
    [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
else
    [new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
end
qDots = [qDots; qDot];
qMatrix = [qMatrix; new_qMatrix];
m = [m; new_m];

% Another spline
% Obtain current EE pose
last_q = qMatrix(end,:);
T = R.fkine(last_q).T;
rpy = tr2rpy(T);
last_yaw = rpy(3);
% Generate points
y_points = [T(2,end), 3.1, 2.97, 2.82, 2.74, 2.25, 1.87, 1.615];
x_points = lspb(T(1,end), -2.55, length(y_points))';
% Obtain the polynomial
p = polyfit(x_points, y_points, length(y_points)-1);
x1 = linspace(T(1,end), -2.55, steps);
f1 = polyval(p, x1);
if abs(f1(1) - T(2,end)) > 0.01
    error('polynomial bad')
end
% Generate the trajectory
trajectory = [x1; f1; zeros(1,steps)];
dist_per_point = abs(1.09*pi - last_yaw)/length(trajectory);
for i=1:length(trajectory)
    theta(:,i) = [0 0 (last_yaw + i*dist_per_point)]';
end
if not_optimise_rmrc
    [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
else
    [new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
end
qDots = [qDots; qDot];
qMatrix = [qMatrix; new_qMatrix];
m = [m; new_m];

% Line
% Obtain current EE pose
last_q = qMatrix(end,:);
T = R.fkine(last_q).T;
rpy = tr2rpy(T);
last_yaw = rpy(3);
% Generate the trajectory
dist_per_point = abs(1.1*pi + last_yaw)/steps;
for i=1:steps
    trajectory(1,i) = (1-s(i))*T(1,4) + s(i)*-3.1; % Points in x
    trajectory(2,i) = (1-s(i))*T(2,4) + s(i)*-0.3; % Points in y
    theta(3,i) = (last_yaw + i*dist_per_point);                 % Yaw angle
end
if not_optimise_rmrc
    [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
else
    [new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
end
qDots = [qDots; qDot];
qMatrix = [qMatrix; new_qMatrix];
m = [m; new_m];

% Sine
% Obtain current EE pose
last_q = qMatrix(end,:);
T = R.fkine(last_q).T;
rpy = tr2rpy(T);
last_yaw = rpy(3);
% Generate the trajectory
dist_per_point = abs(1.1*pi + last_yaw)/steps;
for i=1:steps
    trajectory(1,i) = (1-s(i))*T(1,4) + s(i)*-0.73; % Points in x
    trajectory(2,i) = T(2,4) + 0.3*sin(i*0.01*exp((i-steps)/800)); % Points in y
    theta(3,i) = last_yaw; % (last_yaw + i*dist_per_point);                 % Yaw angle
end
if not_optimise_rmrc
    [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
else
    [new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
end
qDots = [qDots; qDot];
qMatrix = [qMatrix; new_qMatrix];
m = [m; new_m];

%% Plot the results
close all
fps = 10000000;
figure(1)
load('final_cartesian_xy_points.mat');
final_cartesian_xy_points = final_cartesian_xy_points.final_cartesian_poses;
check_traj(final_cartesian_xy_points)
R.plot(qMatrix,'trail','r-', 'fps', fps, 'workspace', [-4, 4 -4 4 -4 4], 'zoom', 1.8)

figure(2) % x,y position of EE and manipulability
subplot(2,1,1)
hold on
plot(final_cartesian_xy_points(1, 1:1000), final_cartesian_xy_points(2,1:1000), 'g.')
plot(final_cartesian_xy_points(1, 1001:2000), final_cartesian_xy_points(2,1001:2000), 'r-.')
plot(final_cartesian_xy_points(1, 2001:3000), final_cartesian_xy_points(2,2001:3000), 'k.')
plot(final_cartesian_xy_points(1, 3001:4000), final_cartesian_xy_points(2,3001:4000), 'r--')
plot(final_cartesian_xy_points(1, 4001:5000), final_cartesian_xy_points(2,4001:5000), 'm.')
hold off
axis padded
title('EE position ')
ylabel('y pos')
xlabel('x pos')

subplot(2,1,2)
hold on
plot(1:1000, m(1:1000,:), 'g.')
plot(1001:2000, m(1001:2000,:), 'r-.')
plot(2001:3000, m(2001:3000,:), 'k.')
plot(3001:4000, m(3001:4000,:), 'r--')
plot(4001:5000, m(4001:5000,:),  'm.')
title('Manipulability')
hold off
refline([0, 1.5])

figure(3) % joint velocity and manipulability
subplot(3,1,1)
hold on
plot(qDots(:, 1), 'r-')
plot(qDots(:, 2), 'g-')
hold off
legend('Joint1','Joint2')
axis padded
title('Prismatic Joint Velocities')
ylabel('Joint velocity')
xlabel('Step')

subplot(3,1,2)
hold on
plot(qDots(:, 3), 'b-')
plot(qDots(:, 4), 'k-')
plot(qDots(:, 5), 'm-')
% plot(qDots(:, 6), 'c-')
% plot(qDots(:, 7), 'm-')
hold off
% legend('Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7')
% legend('Joint3', 'Joint4', 'Joint5', 'Joint6')
legend('Joint3', 'Joint4', 'Joint5')
axis padded
title('Revolute Joint Velocities ')
ylabel('Joint velocity')
xlabel('Step')

subplot(3,1,3)
hold on
plot(1:1000, m(1:1000,:), 'g.')
plot(1001:2000, m(1001:2000,:), 'r-.')
plot(2001:3000, m(2001:3000,:), 'k.')
plot(3001:4000, m(3001:4000,:), 'r--')
plot(4001:5000, m(4001:5000,:),  'm.')
title('Manipulability')
hold off
refline([0, 1.5])

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

function save_stuff()
    all_transforms = R.fkine(qMatrix).T;
    xy_points = all_transforms(1:3,4,:);
    final_cartesian_xy_points = zeros(3,5000);
    for i=1:5000
        final_cartesian_xy_points(:,i) = store_here_xy(:,1,i);
    end
end

function [qMatrix, m, qdot] = RMRC_with_optim(trajectory, theta, deltaT, q0, R)

    % Track the trajectory with RMRC
    qMatrix = zeros(length(trajectory), R.n);
    m = zeros(length(trajectory),1);             % Array for Measure of Manipulability
    qdot = zeros(length(trajectory)-1, R.n);          % Array for joint velocities
    
    qMatrix(1,:) = q0; % Initialise first waypoint

    % Numerical calculation for Jacobian derivative wrt joint pos    
    h = 10^-4;
    delta_matrix = diag(h+zeros(R.n,1));
    numerical_dm_dq_ = nan(1,R.n); % Init Jacobian deriv matrix
    
    SCALE_NULL_VEL = 5;
    min_allowable_weight = power(10,-5);
    beta = 0.01;
    tau = 1.5;
    primary_weight = zeros(1,R.n);
    secondary_weight = zeros(1,R.n);
    
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
        beta_power = -0.67 + (m(i) + min_allowable_weight) / tau;
        arm_weight = power(beta, beta_power);
        if arm_weight > 1
            arm_weight = min_allowable_weight + ((arm_weight-0.01)/(15-0.01))*(0.99-min_allowable_weight);
        end
        platform_weight = 1.0 - arm_weight;
%         if platform_weight < 0
%             disp(string(arm_weight))
%             platform_weight = min_allowable_weight;
%         end
%         
        for j=1:R.n
            if j < 3
                primary_weight(1,j) = platform_weight;
                secondary_weight(1,j) = 1;
            else
                primary_weight(1,j) = arm_weight;
                secondary_weight(1,j) = min_allowable_weight;
            end
        end
        
        
        pinvJ_primary = pinv_weighted(primary_weight, J);
        pinvJ_secondary = pinv_weighted(secondary_weight ,J);
        % End code for weight selection
        
%         pinvJ = pinv_weighted(J); % MP Inverse
        
        
        for k = 1:R.n % joints
            
            % Find the 'next; and 'previous' Jacobian given a movement in
            % the independent (joint) variable and numerically calculate
            % the jacobian wrt the independent variable
            jac_p = R.jacob0(curr_q + delta_matrix(k,:));
            jac_p = [jac_p(1:2,:); jac_p(6,:)];
            jac_m = R.jacob0(curr_q - delta_matrix(k,:));
            jac_m = [jac_m(1:2,:); jac_m(6,:)];
            partialJ_partialqk = (jac_p - jac_m)/(2*h);     
            
            numerical_dm_dq_(k) = m(i)*trace(partialJ_partialqk*pinvJ_secondary);
        
        end

        
        % Update next joint state based on joint velocities
%         (eye(R.n)-pinvJ*J)*numerical_dm_dq_'
%         (pinvJ*xdot)
        qdot(i,:) = (pinvJ_primary*xdot) + SCALE_NULL_VEL*(eye(R.n)-pinvJ_secondary*J)*numerical_dm_dq_';
        qMatrix(i+1,:) = curr_q + deltaT*qdot(i,:);

    end
    % Manipulabilityu at final state
    J = R.jacob0(qMatrix(end,:));
    J = [J(1:2, :); J(6, :)];
    j_man = J(:,3:end);
    m(end) = sqrt(det(j_man*j_man'));
    

end

function [qMatrix, m, qdot] = RMRC_traj_tracking(trajectory, theta, deltaT, q0, R)
    % Track the trajectory with RMRC
    qMatrix = zeros(length(trajectory), R.n);
    m = zeros(length(trajectory),1);             % Array for Measure of Manipulability
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
    J = R.jacob0(qMatrix(end,:));
    J = [J(1:2, :); J(6, :)];
    j_man = J(:,3:end);
    m(end) = sqrt(det(j_man*j_man'));
end

function check_traj(trajectory)
% Check trajectory
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'k.')

end

function [x_rot, y_rot] = rotate_points_theta_rad(x, y, theta)
    % create a matrix of these points, which will be useful in future calculations
    v = [x;y];
    % choose a point which will be the center of rotation
    x_center = x(round(length(x)/2));
    y_center = y(round(length(y)/2));
    % create a matrix which will be used later in calculations
    center = repmat([x_center; y_center], 1, length(x));
    % define a 40 degree counter-clockwise rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    % do the rotation...
    s = v - center;     % shift points in the plane so that the center of rotation is at the origin
    so = R*s;           % apply the rotation about the origin
    vo = so + center;   % shift again so the origin goes back to the desired center of rotation
    % this can be done in one line as:
    % vo = R*(v - center) + center
    % pick out the vectors of rotated x- and y-data
    x_rot = vo(1,:);
    y_rot = vo(2,:);
    % make a plot
%     plot(x_rot, y_rot, 'r-', x_center, y_center, 'bo');
%     axis equal
end