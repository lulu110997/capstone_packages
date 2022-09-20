clc
close all
max_manip = 2; % Max manipulability that the system can reach
beta = [0.07 0.3, 0.66, 0.77]; % Parameter that changes whether the function approaches inf or zero
tau = [0.2 0.7 0.4 0.1]; % Allowable manipulability, changes the max possible 'y' value (ie weight) that can be reached
min_allowable_weight = power(10,-5);
%
x = 0:0.01:max_manip;
manipulability = 0:0.01:max_manip;
for i=1:length(beta)
beta_power = (manipulability + min_allowable_weight)/tau(i);
y = beta(i).^beta_power;
hold on
plot(x,y)
end
legend('Experiment 1: beta='+string(beta(1)) + ', tau='+string(tau(1)),...
       'Experiment 2: beta='+string(beta(2)) + ', tau='+string(tau(2)),...
       'Expetiment 3: beta='+string(beta(3)) + ', tau='+string(tau(3)));
%%
x_points = [3 2.8 2.5 2.2 1.7 1.5 1.4 1.35 1.3 1 0.9 0.8 0.2 0.1 0.01];
y_points = [0.001 0.01 0.01 0.04 0.1 0.15 0.2 0.3 0.4 0.8 0.85 0.9 1 1 1];
% y_points = lspb(T(2,end), 3, 5)';
% interpolated_points = fnplt(cscvn([x_points; y_points]));
% Obtain the polynomial
f = fit(x_points',y_points','exp1')
% plot(f)