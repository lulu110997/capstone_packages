classdef approximating_jacobians_linux
    properties
        qs_ = [-0.87266, -1.2217, 1.2217, 0, 0, 0]
        h_ = 0.1
        teach_ = 0
    end
    methods
        function self = approximating_jacobians(h, teach, qs)
            % Set teach = 1 to turn on teach mode
            if nargin == 1
                self.h_ = h;
            elseif nargin == 2
                self.h_ = h;
                self.teach_ = teach;
            elseif nargin == 3
                self.h_ = h;
                self.qs_ = qs;
                self.teach_ = teach;
            end
            j0 = {
                '-0.4343   -0.2957    0.0744    0.0744   -0.0593         0'
                '-0.6988    0.3524   -0.0886   -0.0886    0.0706         0'
                '-0.0000   -0.7819   -0.5723   -0.0000    0.0000         0'
                ' 0.0000   -0.7660   -0.7660   -0.7660   -0.0000   -0.7660'
                '-0.0000   -0.6428   -0.6428   -0.6428   -0.0000   -0.6428'
                ' 1.0000    0.0000    0.0000    0.0000   -1.0000    0.0000'
                ''};
            disp('The jacobian of the UR10 at joint [0, 0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0] is')
            fprintf('\n    %s', j0{:})
        end
        
        function [j0_actual, j0_apprx, je] = jacobian_of_mobman(self, qs)
            % This function approximates the Jacobian of a UR10 manipulator
            % using FDM It outputs the actual Jacobian and then the
            % approximated Jacobian. It also returns the Jacobian in the EE
            % frame as the third argument in the array
            if nargin < 2
                qs = [0, 0, 0, 0, -0.5230, 1.2215, 2.9665, 1.5705, 0];
            end
            
            % Mobile base modelled with 2 prismatic links and one revolute link
            L1 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0); % REVOLUTE Link around z
            L2 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link alond y
            L3 = Link('theta', -pi/2, 'a', 0, 'alpha', -pi/2, 'qlim', [-5, 5], 'offset', 0); % PRISMATIC Link along x

            % Arm manipulator. The base link hass and offset of -pi/2 to correct for the
            % difference of the original base coord frame to the change in coord frame 
            % due to the first 3 links
            L4 = Link('d',0.1273,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]), 'offset', pi/2); 
            L5 = Link('d',0,'a',-0.6127,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset',0);
            L6 = Link('d',0,'a',-0.5723,'alpha', 0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L7 = Link('d',0.163941,'a',0,'alpha', pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L8 = Link('d',0.1157,'a',0,'alpha', -pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L9 = Link('d',0.0922,'a',0,'alpha', 0,'qlim',deg2rad([-360,360]), 'offset', 0);
            R = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8 L9],'name','ur10e');

            j0_arm = R.jacob0(qs);
            j0_arm(1:6, 4:9);

            if self.teach_ == 1
                close all;
                R.plot(qs, 'workspace',[-6 6 -6 6 -3 3], 'scale', 0.2)
                R.teach(qs)
            end
            
            j0_actual = R.jacob0(qs);
            j0_apprx = self.fdm(R, qs);
            je = R.jacobe(qs);
        end
        
        function [j0_actual, j0_apprx, je] = jacobian_of_UR10(self)
            % This function approximates the Jacobian of a UR10 manipulator
            % using FDM It outputs the actual Jacobian and then the
            % approximated Jacobian. It also returns the Jacobian in the EE
            % frame as the third argument in the array
            
            qs = self.qs_;
            
            L1 = Link('d',0.1273,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', deg2rad(180));
            L2 = Link('d',0,'a',-0.6127,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',-0.5723,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.163941,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L5 = Link('d',0.1157,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0922,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            R = SerialLink([L1 L2 L3 L4 L5 L6],'name','ur10e');
            
            if self.teach_ == 1
                close all;
                R.plot(qs)
                R.teach(qs)
            end
            
            j0_actual = R.jacob0(qs);
            j0_apprx = self.fdm(R, qs);
            je = R.jacobe(qs);
        end
        
        function j0_approx = fdm(self, R, qs)
            % Use finite difference method to approximate Jacobians for a
            % range of robots. R is the robot model, qs is the starting
            % joint position 
            
            % Matrix of small changes for each joint
            no_cols = size(qs,2);
            dq = diag(self.h_+zeros(no_cols,1));
            
            % Obtain the forward kinematics
            qs_fkine = R.fkine(qs);
            
            % Obtain the xyz locations and rpy values at q(h+1)
            qs_fkine_p = R.fkine(qs + dq(1,:));
            xyz_p = qs_fkine_p.t;
            rpy_p = tr2rpy(qs_fkine_p.R*inv(qs_fkine.R));

            % Obtain the xyz locations and rpy values at q(h-1)
            qs_fkine_m = R.fkine(qs - dq(1,:));
            xyz_m = qs_fkine_m.t;
            rpy_m = tr2rpy(qs_fkine_m.R*inv(qs_fkine.R));

            % Finds how a small change in each local joint affects the EE spatial velocity   
            j0_approx =  ([xyz_p; rpy_p'] - [xyz_m; rpy_m'])/(2*self.h_);
            for i = 2:no_cols  
                qs_fkine_p = R.fkine(qs + dq(i,:));
                xyz_p = qs_fkine_p.t;
                rpy_p = tr2rpy(qs_fkine_p.R*inv(qs_fkine.R));

                qs_fkine_m = R.fkine(qs - dq(i,:));
                xyz_m = qs_fkine_m.t;
                rpy_m = tr2rpy(qs_fkine_m.R*inv(qs_fkine.R));

                j0_approx =  cat(2, j0_approx, ([xyz_p; rpy_p'] - [xyz_m; rpy_m'])/(2*self.h_));
            end
        end


    end
end