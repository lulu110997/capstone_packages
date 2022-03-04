clear all; clc; close all
h = 0.1;
dq = diag(h+zeros(6,1));
qs = [-0.6283 -1.2566 0.6283 -1.2566 -1.2566 0];
L1 = Link('d', 0.1807, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L2 = Link('d', 0,'a', -0.6127, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);
L3 = Link('d', 0, 'a', -0.57155, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);
L4 = Link('d', 0.17415, 'a', 0, 'alpha', pi/2, 'qlim', [-2*pi, 2*pi]);
L5 = Link('d', 0.11985, 'a', 0, 'alpha', -pi/2, 'qlim', [-2*pi, 2*pi]);
L6 = Link('d', 0.11655, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);
R = SerialLink([L1 L2 L3 L4 L5 L6],'name','ur10e');

R.plot(qs - dq)
R.teach

qs_fkine = R.fkine(qs);

qs_fkine_p = R.fkine(qs + dq(1,:));
xyz_p = qs_fkine_p(1:3,4);
rpy_p = tr2rpy(qs_fkine_p(1:3,1:3)*inv(qs_fkine(1:3,1:3)));

qs_fkine_m = R.fkine(qs - dq(1,:));
xyz_m = qs_fkine_m(1:3,4);
rpy_m = tr2rpy(qs_fkine_m(1:3,1:3)*inv(qs_fkine(1:3,1:3)));

jacob0_apprx =  ([xyz_p; rpy_p'] - [xyz_m; rpy_m'])/(2*h);
    
for i = 2:6    
    qs_fkine_p = R.fkine(qs + dq(i,:));
    xyz_p = qs_fkine_p(1:3,4);
    rpy_p = tr2rpy(qs_fkine_p(1:3,1:3)*inv(qs_fkine(1:3,1:3)));

    qs_fkine_m = R.fkine(qs - dq(i,:));
    xyz_m = qs_fkine_m(1:3,4);
    rpy_m = tr2rpy(qs_fkine_m(1:3,1:3)*inv(qs_fkine(1:3,1:3)));
    
    jacob0_apprx =  cat(2, jacob0_apprx, ([xyz_p; rpy_p'] - [xyz_m; rpy_m'])/(2*h));
end

jcb0 = R.jacob0(qs)
jacob0_apprx
%%
clear
clc
syms l 
syms piontwo
syms negpiontwo
syms zero
% DH parameters
L1 = Link('d', l, 'a', zero, 'alpha', piontwo);
L2 = Link('d', zero,'a', l, 'alpha', zero);
L3 = Link('d', zero, 'a', l, 'alpha', zero);
L4 = Link('d', l, 'a', zero, 'alpha', piontwo);
L5 = Link('d', l, 'a', zero, 'alpha', negpiontwo);

rob = SerialLink([L1 L2 L3 L4 L5],'name','ur10e');

cGen = CodeGenerator(rob, 'mfun', 1);
jac = cGen.genjacobian;
