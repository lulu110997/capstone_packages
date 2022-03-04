%% Jacobian and Jacobian time derivative computations
% Based on the paper 'Analytical Method for Differentiation of Robot
% Jacobian' by Jaesuk Rhee, Bum-Joo Lee published 20 March 2017
clc
clear

% qs = [-1.1310   -1.5080    1.8850   -0.2513         0         0];
% qd = [-1.86   2.37    2.15   -0.2513         0.1         0.1];

qs = -5 + (5+5)*rand(1,6); % Generate random joint positions within the limit
qdl = -2 + (2+2)*rand(1,3); % Generate random linear joint velocities between -2 and 2
qdw = -0.5 + (0.5+0.5)*rand(1,3); % Generate random linear joint velocities between -0.5 and 0.5
qd = [qdl qdw];

r = UR10;
RTB_jacobian =  r.model.jacob0(qs);

% jacobian computation
z_0_0 = [zeros(2,1); 1]; % Z axis unit vector of first frame
d_0_6 = transl(r.model.fkine(qs)); % Origin vector to EE wrt base frame 

analytic_jac_ = [cross(z_0_0, d_0_6); z_0_0]; % jacobian of first joint

% Obtain the first homogenous transform from base frame to first frame
A_prev = r.model.links(1).A(qs(1));

% Variables to store values to compute the jacobian time derivative
ds = []; % Matrix to store origin vector for each trans mat
zs = []; % Matrix to store z unit vector for each rot mat
omegas = [[] [z_0_0*qd(1)]]; % Calculate first omega computation
                             % here instead of inside for loop due to
                             % syntax issues

for i = 2:6
        
    % Obtain the CURRENT homogenous transform for the ith link from the dh params
    % ie: this is the transformation matrix from i-1 to i (T_i-1_i)
    theta = qs(i); % theta: joint angle
    d = r.model.links(i).d; % z offset
    a = r.model.links(i).a; % x offset
    alphaa = r.model.links(i).alpha; % rot about x axis
    T_i = trotz(theta)*transl(0,0,d)*transl(a,0,0)*trotx(alphaa);
    
    % Obtain the transform from world frame to current i-1 frame (T_0_i-1)
    T_0_i_prev = A_prev;
    
    % Obtain the z axis of the i-1th coordinate frame wrt the base frame
    % from the rotation matrix
    z0_i_prev = T_0_i_prev(1:3,1:3)*[0; 0; 1];
    
    % Obtain the displacement from the i-1th coordinate frame to the nth
    % coordinate frame wrt to the base frame
    displ = d_0_6 - T_0_i_prev(1:3,4);
    
    analytic_jac_ = [analytic_jac_ [cross(z0_i_prev, displ); z0_i_prev]]; % jacobian of the ith joint
    
    % Obtain the transformation matrix of the next frame wrt to the base
    % frame (ie T_0_i-1 = T_0_i-2 * T_i-2_i-1)
    A_prev = A_prev*T_i; 

    % For calculating Jacobian derivative
    zs = [zs z0_i_prev];
    ds = [ds T_0_i_prev(1:3,4)];
    if i > 2 % To avoid index error cause of the i-2 index and ML starts 
             % indexing at 1
        omegas = [omegas [omegas(:,i-2) + zs(:,i-2)*qd(i-1)]];
    end
end

errr = abs(max(max(analytic_jac_ - RTB_jacobian)));
if errr > 0.001
    errr
    analytic_jac_
    RTB_jacobian
    error('Computed jacobian does not match with RTB')
    return
end

analytic_jac_dt_ = [];

% Computation of Jacobian time derivative
betaa = analytic_jac_(1:3,i)*qd(i);

for i = 6:-1:2
    zd0_i_prev = cross(omegas(:,i-1), zs(:,i-1));
    alphaa = [0, 0, 0]';
    
    for j = 1:(i-1)
        
        if j-1 == 0
            z0_j_prev = z_0_0;
            d0_i_prev = 0;
        else
            z0_j_prev = zs(:,j-1);
            d0_i_prev = ds(:,i-1);
        end
        
        alphaa = alphaa + cross(z0_j_prev, (d_0_6 - d0_i_prev)*qd(j));
    end
    
    analytic_jac_dt_ = [analytic_jac_dt_ [cross(zd0_i_prev, (d_0_6 - ds(:,i-1))) + cross(zs(:,i-1), alphaa+betaa); zd0_i_prev]];
    betaa = betaa + analytic_jac_(1:3,i-1)*qd(i-1);
end

analytic_jac_dt_ = flip([analytic_jac_dt_ [cross(z_0_0, betaa); [0 0 0]']],2);


% Numerical calculation for Jacobian derivative
h = 10^-8;

delta_matrix = diag(h+zeros(6,1));
numerical_jac_dt_ = zeros(6,6); % Init Jacobian deriv matrix

for i = 1:6 % row

    for j = 1:6 % col
        
        summ = 0;
        
        for k = 1:6 % partial deriv of jac wrt joints
        
            partialJ_partialqk = (r.model.jacob0(qs + delta_matrix(k,:)) - r.model.jacob0(qs - delta_matrix(k,:)))/(2*h);
            summ = summ + partialJ_partialqk(i,j)*qd(k);
        
        end
       
        numerical_jac_dt_(i,j) = summ;
        
    end

end

disp('Jacobian time derivatives')
% analytic_jac_
% numerical_jac_dt_
max_error_between_num_and_analysis = abs(max(max(abs(analytic_jac_dt_ - numerical_jac_dt_))))
matr_err = numerical_jac_dt_ - analytic_jac_dt_

disp('Products of Jd*qd')
% numerical_jacob_dot_ = numerical_jac_dt_*qd'
% analytic_jac_dot_ = analytic_jac_dt_*qd'
% RTB_jacob_dot = r.model.jacob_dot(qs, qd)

num_analy_err = abs(max(numerical_jac_dt_*qd' - analytic_jac_dt_*qd'))
analy_rtb_err = abs(max(analytic_jac_dt_*qd' - r.model.jacob_dot(qs, qd)))
num_rtb_err = abs(max(numerical_jac_dt_*qd' - r.model.jacob_dot(qs, qd)))

% j1 = r.model.jacob0(qd*0.1);
% j2 = r.model.jacob0(qd*-0.1);
% jd = (j1-j2)/(0.2);
% j_dot = jd*qd';
% jd_qd = r.model.jacob_dot(qs, qd);