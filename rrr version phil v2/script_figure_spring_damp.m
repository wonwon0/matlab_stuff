load('theta_saved_force.mat')
theta_saved_force=theta_saved;
load('theta_saved.mat')
close all
plot(theta_saved(:,3), 'LineWidth',2)
hold on
plot(theta_saved_force(:,3), 'LineWidth',2)
legend('Sliding algorithm','Virtual spring-damper')
xlabel('step')
ylabel('angular position (rad)')