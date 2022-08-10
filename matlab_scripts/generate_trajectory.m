% Simulation parameters
t = 2;             % Total time (s)
deltaT = 0.002;      % Corresponds with the 500Hz control frequency
steps = t/deltaT;   % No. of steps for simulation
theta = zeros(3,4995);         % Array for roll-pitch-yaw angles
trajectory = zeros(3,4995);             % Array for x-y-z trajectory

% Create the sinusoidal trajectory
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    trajectory(1,i) = (1-s(i))*2.8 + s(i)*1.5; % Points in x
    trajectory(2,i) = 1.6 + 0.13*sin(i*0.013); % Points in y
    theta(3,i) = pi/9;                 % Yaw angle
end
[trajectory(1,:), trajectory(2,:)] = rotate_points_theta_rad(trajectory(1,:), trajectory(2,:), pi/3);

% Generate a trajectory using by creating a polynomial from a set of points
y_points = [trajectory(2,end), 1.1, 1.35, 1.4, 1.47, 1.5, 1.8, 2.5, 2.8, 2.879, 3, 3.08];
x_points = lspb(trajectory(1,end), -0.05, length(y_points))';
% Obtain the polynomial
p = polyfit(x_points, y_points, length(y_points)-1);
x1 = linspace(T(1,end), 0, steps);
f1 = polyval(p, x1);

% Generate the trajectory
trajectory = [x1; f1; zeros(1,steps)];
dist_per_point = ((0.7*pi)/2 - last_yaw)/length(trajectory);
for i=1:length(trajectory)
    theta(:,i) = [0 0 (last_yaw + i*dist_per_point)]';
end
% [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
[new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
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
% [new_qMatrix, new_mqDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
[new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
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
% [new_qMatrix, new_m, qDot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
[new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
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
% [new_qMatrix, new_m, qdot] = RMRC_traj_tracking(trajectory, theta, deltaT, last_q, R);
[new_qMatrix, new_m, qDot] = RMRC_with_optim(trajectory, theta, deltaT, last_q, R);
qDots = [qDots; qDot];
qMatrix = [qMatrix; new_qMatrix];
m = [m; new_m];

plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'k.')

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