L=[0.2755 0.290 0.390 0.0098;
    0.2755 0.290 0 0.0098];
pose_act=[0.2880    0.3920    0.2754];
theta_act=cin_inv(pose_init,[0 -pi/2 -pi/2],L(1,:))

vec_norm_coude=[1 2 3]/norm([1 2 3]);
fprintf('solution philippe: \n\n\n');
jacob_eff=jacobian_loc
