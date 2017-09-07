a=[0 -425 -392.25 0 0 0];
b=[89.159 0 0 109.15 94.65 82.3];
dh_theta=[0  -pi/2 0 -pi/2 0 0];
alpha=[pi/2 0 0 pi/2 -pi/2 0];
theta_act=[0 0 0 0 0 0];
[p, euler]=cin_dir_6ddl(theta_act, a, b, alpha, dh_theta)