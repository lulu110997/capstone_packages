clc
clear
close
teach_ = 0;
qs = [0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0];
% qs = [0.3, -1.2, 0, -0.5230, 1.2215, 2.9665, 1.5705, 0];

% Mobile base modelled with 2 prismatic links
L1 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link along y
L2 = Link('theta', -pi/2, 'a',0 , 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link along x

% Arm manipulator. The base link hass and offset of -pi/2 to correct for the
% difference of the original base coord frame to the change in coord frame 
% due to the first 3 links
L3 = Link('d',0.1273,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]), 'offset', pi/2); 
L4 = Link('d',0,'a',-0.6127,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset',0);
L5 = Link('d',0,'a',-0.5723,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset', 0);
L6 = Link('d',0.163941,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
L7 = Link('d',0.1157,'a',0,'alpha', -pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L8 = Link('d',0.0922,'a',0,'alpha', 0,'qlim',deg2rad([-360,360]), 'offset', 0);
R = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8],'name','emu');

R.base = trotx(-pi/2);

j0_arm = R.jacob0(qs);

if teach_ == 1
    close all;
    R.plot(qs, 'workspace',[-6 6 -6 6 -3 3], 'scale', 0.2)
    R.teach(qs)
end

j0_actual = R.jacob0(qs);
je = R.jacobe(qs)
