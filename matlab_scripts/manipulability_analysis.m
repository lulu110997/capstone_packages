% % maximising manipulability doesnt happen instantly --> could take x
% % iterations before converging to maximum
%     % secondary task might be happening slowly
%     % self motion not optimised fast enough to the primary task
% % use a threshold so that the norm of joint velocity in the joint space is
% % almost zero (null space motion is close to almost complete)
% 
% clear
% clc
% close 
% 
% % Define different joint configs
% qz = [0, 0, 0, 0, 0];
% qs_m0 = [-1.2210   -0.6975   -1.0465   -0.3485   -0.3485]; % Starting joint with max_m = 1.7048
% qs_m1 = [-2.0323   -0.9040    2.5835   -2.2074   -1.1017]; % Same EE pose as above but with m = 0.7044
% qs_m2 = [-2.0957    0.1364    0.3451   -2.4400    0.3921]; % As above but with m = 0.9619
% qs_m3 = [-2.1614    0.1527   -2.8138    2.7105   -1.5500]; % As above but with m = 0.3421
% qs_m4 = [0.5235    0.2617         0    0.2617    1.0470]; % Random config with m = 0.5845
% qs_m5 = [-0.5906    0.2155   -2.0735    0.5742   -1.2358]; % Random config
% 
% % Create a 5 DoF 2D manipulator
% L1 = Link('d',0,'a',0.6127,'alpha', 0);
% L2 = Link('d',0,'a',0.5723,'alpha', 0);
% L3 = Link('d',0,'a',0.4,'alpha', 0);
% L4 = Link('d',0,'a',0.5283,'alpha', 0);
% L5 = Link('d',0,'a',0.6,'alpha', 0);
% R = SerialLink([L1 L2 L3 L4 L5],'name','redundant planar robot');
% 
% while true
%     qs = -1.57 + (2*1.57+1.57)*rand(1,R.n); % Generate random joint positions within the limit
%     manip = R.maniplty(qs, 'dof', [1 1 0 0 0 1])
% 
%     if manip < 0.5
%         manip
%         break
%     end
% 
% end
% % qs = [0.5521    1.3594    1.3102    0.3252    0.2380]
% % qe = -1 + (1+0)*rand(1,R.n) % Generate random joint positions within the limit %qs_m0;
% qe = qs
% R.plot(qs, 'workspace',[-3 3 -3 3 -1 1])
% 
% % find_max_manipulability(R)
% % find_different_qs(R, qs_m0)
% self_motion(R, qs, qs)
% % nullspace_projection(R, qs_m3, qs_m0)
% 
find_max_manipulability(R.model)

function self_motion(R, qs, qe)

    num_joints = R.n;
%     task_space = 3;
    steps = 100;
    delta_t = 0.5;
    manipulator_history = zeros(steps,1);
    q_matrix = nan(steps,num_joints);
    q_matrix(1,:) = qs;
    
    % Numerical calculation for Jacobian derivative wrt joint pos    
    h = 10^-4;
    delta_matrix = diag(h+zeros(num_joints,1));
%     numerical_jac_dq_ = nan(task_space,num_joints); % Init Jacobian deriv matrix
    numerical_dm_dq_ = nan(1,num_joints); % Init Jacobian deriv matrix
    
    if all(qs == qe)
        qd_wp = zeros(100,num_joints);
        x_wp = zeros(3,steps);
    else
        [~, qd_wp] = jtraj(qs, qe, steps);
        x_wp = zeros(3,steps);
        x1 = R.fkine(qs).t;
        x2 = R.fkine(qe).t;
        s = lspb(0,1,steps);                                 % Create interpolation scalar
        for i = 1:steps
            x_wp(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
        end
    end
    
    for step = 1:steps-1
        
%         previous_jac_dq = zeros(task_space,num_joints);
        
        for k = 1:num_joints % joints
            
            % Obtain the current joint config
            curr_q = q_matrix(step,:);
            
            % Obtain the current manipulability, 2D Jacobian for the 
            % current joint config and its pseudoinverse
            curr_jac = R.jacob0(curr_q);
            curr_jac = [curr_jac(1:2,:); curr_jac(6,:)];
            curr_jac_pinv = pinv(curr_jac);
            curr_mani = sqrt(det(curr_jac*transpose(curr_jac)));
            
            % Find the 'next; and 'previous' Jacobian given a movement in
            % the independent (joint) variable and numerically calculate
            % the jacobian wrt the independent variable
            jac_p = R.jacob0(curr_q + delta_matrix(k,:));
            jac_p = [jac_p(1:2,:); jac_p(6,:)];
            jac_m = R.jacob0(curr_q - delta_matrix(k,:));
            jac_m = [jac_m(1:2,:); jac_m(6,:)];
            partialJ_partialqk = (jac_p - jac_m)/(2*h);     
            
            numerical_dm_dq_(k) = curr_mani*trace(partialJ_partialqk*curr_jac_pinv);
        
        end
        
        % If using RMRC
        xdot = (x_wp(:,step+1) - x_wp(:,step))/delta_t; % Calculate velocity at discrete time step
        q_d = pinv(curr_jac)*xdot + (eye(num_joints)-curr_jac_pinv*curr_jac)*numerical_dm_dq_';
        
        %If not using RMRC (ie using jtraj)
%         q_d = qd_wp(step,:)' + (eye(num_joints) - curr_jac_pinv*curr_jac) * numerical_dm_dq_';
        
        q_matrix(step+1,:) = delta_t*q_d' + q_matrix(step,:);
        manipulator_history(step) = curr_mani;
        
    end
    
    R.plot(q_matrix, 'delay', 0.1)
    figure
    plot(1:steps, manipulator_history)
    title('manipulability history')
    final_q = R.getpos
    mannn = R.maniplty(R.getpos, 'dof', [1 1 0 0 0 1])

end

function find_different_qs(R, q)
% Finding different joint configs and the corresponding manipulability
% value

    T_m = R.fkine(q);
    qn = R.ikine(T_m, 'mask', [1 1 0 0 0 1])
    R.plot(qn)
    jac = R.jacob0(qn);
    jac = [jac(1:2,:); jac(6,:)];
    m = sqrt(det(jac*transpose(jac)))

end

function find_max_manipulability(R)
% Finding the maximum manipulability value

    max_m = 0;
    max_q = zeros(1,R.n);
    step = 0.1745; % 10 deg increments
    min_q_lim = 0;
    max_q_lim = 3.14;
%     for q1 = min_q_lim:step:max_q_lim
        for q2 = -1.57:step:0
            for q3 = 0:step:1.57
                for q4 = min_q_lim:step:max_q_lim
                    for q5 = min_q_lim:step:max_q_lim
                        jac = R.jacob0([0, q2, q3, q4, q5, 0]);
%                         jac = [jac(1:2,:); jac(6,:)]; % For Planar
%                         manipulators
                        m = sqrt(det(jac*transpose(jac)));
                        if m > max_m
                            max_m = m;
                            max_q = [0, q2, q3, q4, q5];
                        end
                    end
                end
            end
        end
%     end
    
    max_m
    max_q
    
end

