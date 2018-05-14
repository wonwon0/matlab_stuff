close all
clear

fig=figure(1);
camlight;
axis vis3d equal;
view(3);
%set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
xlabel('x')
ylabel('y')
zlabel('z')
mat_ligne_x=[1 2];
mat_ligne_y=[1 2];
mat_ligne_z=[1 2];
h_robot_lines = line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','b');
dh_eff = dh_UR5();
Robot_Pose_j = [0 -pi/2 -pi/2 pi/2 pi/2 0];
Robot_Pose_j = [-1,-2.5,2,0,1,0];
%Robot_Pose_j = [0 -pi/2 0 -pi/2 0 0];
[h_robot_lines] = display_robot_UR5(Robot_Pose_j, dh_eff, h_robot_lines);
drawnow limitrate
rot = [0 0 0];
dir = [0 0 1];
[Robot_Pose, ~] = cin_dir_6ddl_v2( Robot_Pose_j, dh_eff);
rot_mat = Robot_Pose(1:3, 1:3);

next_rotation_mat =  eul2rotm(rot, 'XYZ') * rot_mat;
next_pose = [next_rotation_mat(1,:) dir(1) + Robot_Pose(1,4);...
             next_rotation_mat(2,:) dir(2) + Robot_Pose(2,4);...
             next_rotation_mat(3,:) dir(3) + Robot_Pose(3,4);...
             0 0 0 1]*1;
next_ang = cin_inv_6ddl(next_pose, dh_eff, Robot_Pose_j);
jac_test=jacob_UR5(Robot_Pose_j, Robot_Pose(1:3,4), dh_eff);
theta_dot_verif = jac_test\ [0 0 0 dir]';
theta_dot = (next_ang - Robot_Pose_j);
theta_dot = theta_dot/norm(theta_dot)
theta_dot_verif = theta_dot_verif/norm(theta_dot_verif)