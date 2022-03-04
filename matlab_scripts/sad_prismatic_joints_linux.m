clc; clear all
h = 0.1;
qs = [0, 0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0];
dq = diag(h+zeros(9,1));

% Using teach mode as some sort of boolean. If 1, plot robot and activate 
% teach mode. Else we can use a negative qlim for prismatic joint since the
% errors only appear when we try to plot and use teach mode on the robot
teach = 1; 

% Mobile base modelled with 2 prismatic links and one revolute link
L1 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0); % REVOLUTE Link around z
L2 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link alond y
L3 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link along x

% Arm manipulator. The base link hass and offset of -pi/2 to correct for the
% difference of the original base coord frame to the change in coord frame 
% due to the first 3 links
L4 = Link('d',0.1273,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]), 'offset', -pi/2); 
L5 = Link('d',0,'a',-0.6127,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset',0);
L6 = Link('d',0,'a',-0.5723,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset', 0);
L7 = Link('d',0.163941,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
L8 = Link('d',0.1157,'a',0,'alpha', -pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L9 = Link('d',0.0922,'a',0,'alpha', 0,'qlim',deg2rad([-360,360]), 'offset', 0);
R = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8 L9],'name','ur10e');

j0_arm = R.jacob0(qs);
j0_arm(1:6, 4:9)

if teach == 1
    R.plot(qs, 'workspace',[-6 6 -6 6 -3 3], 'scale', 0.2)
    R.teach(qs)
end