clear
clc
close
q_dots_1_2 = load('q_dots_1_2_restrictred.mat');
q_dots_1_4 = load('q_dots_1_4_restrictred.mat');
q_dots_eq = load('q_dots_equal_weight.mat');
q_dots_no_optim = load('q_dots_no_optim.mat');

m_1_2 = load('m_1_2_restrictred.mat');
m_1_4 = load('m_1_4_restrictred.mat');
m_eq = load('m_equal_weight.mat');
m_no_optim = load('m_no_optim.mat');

f = figure(3); % joint velocity and manipulability
f.WindowState = 'maximized';

%%
clc
f = figure(1); % joint velocity and manipulability
f.WindowState = 'maximized';
count = 1;
param_used = exp3_params;
for i=1:10
    
    subplot(5,2,i)
    xlim([0, 1.5])
    ylim([0, max(abs(param_used.qDots(:,count)))+0.5])
    if i < 5
        title('Prismatic Joint')
    else
        title('Revolute Joint')
    end
    xlabel('Manipulability')
    ylabel('Joint Velocities')
    hold on
    
    if mod(i,2) ~= 0
        scatter(param_used.m, abs(param_used.qDots(:,count)))
    else
        scatter(exp3_k4.m, abs(exp3_k4.qDots(:,count)), 'red')
        count = count + 1;
    end
    
end
sgtitle('Joint Velocities Using Different Gains for Null Velocity') 


%%
subplot(3,3,1)
hold on
plot(abs(q_dots_no_optim.qDots(:, 1)), 'r-')
plot(abs(q_dots_eq.qDots(:, 1)), 'g-')
plot(abs(q_dots_1_2.qDots(:, 1)), 'b-')
plot(abs(q_dots_1_4.qDots(:, 1)), 'm-')
hold off
% axis padded
title('Joint 1 Velocities')
ylabel('Joint velocity')
xlabel('Step')
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')

subplot(3,3,2)
hold on
plot(abs(q_dots_no_optim.qDots(:, 2)), 'r-')
plot(abs(q_dots_eq.qDots(:, 2)), 'g-')
plot(abs(q_dots_1_2.qDots(:, 2)), 'b-')
plot(abs(q_dots_1_4.qDots(:, 2)), 'm-')
hold off
title('Joint 2 Velocities')
ylabel('Joint velocity')
xlabel('Step')
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')

subplot(3,3,3)
hold on
plot(abs(q_dots_no_optim.qDots(:, 3)), 'r-')
plot(abs(q_dots_eq.qDots(:, 3)), 'g-')
plot(abs(q_dots_1_2.qDots(:, 3)), 'b-')
plot(abs(q_dots_1_4.qDots(:, 3)), 'm-')
hold off
title('Joint 3 Velocities')
ylabel('Joint velocity')
xlabel('Step')
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')

subplot(3,3,4)
hold on
plot(abs(q_dots_no_optim.qDots(:, 4)), 'r-')
plot(abs(q_dots_eq.qDots(:, 4)), 'g-')
plot(abs(q_dots_1_2.qDots(:, 4)), 'b-')
plot(abs(q_dots_1_4.qDots(:, 4)), 'm-')
hold off
title('Joint 4 Velocities')
ylabel('Joint velocity')
xlabel('Step')
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')

subplot(3,3,5)
hold on
plot(abs(q_dots_no_optim.qDots(:, 5)), 'r-')
plot(abs(q_dots_eq.qDots(:, 5)), 'g-')
plot(abs(q_dots_1_2.qDots(:, 5)), 'b-')
plot(abs(q_dots_1_4.qDots(:, 5)), 'm-')
hold off
title('Joint 5 Velocities')
ylabel('Joint velocity')
xlabel('Step')
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')

ax = legend('Without optimisation', 'Optimised with equal weighting', 'Optimised with limited movement for joint 1 and 2', 'Optimised with limited movement for joint 1 and 4',...
    'location','southoutside');

subplot(3,3,6)
hold on
plot(m_no_optim.m, 'r-')
plot(m_eq.m, 'g-')
plot(m_1_2.m, 'b-')
plot(m_1_4.m, 'm-')
title('Manipulability')
ylabel('Manipuability')
xlabel('Step')
hold off
xline(1000, 'LineWidth',2, 'HandleVisibility','off')
xline(2000, 'LineWidth',2, 'HandleVisibility','off')
xline(3000, 'LineWidth',2, 'HandleVisibility','off')
xline(4000, 'LineWidth',2, 'HandleVisibility','off')