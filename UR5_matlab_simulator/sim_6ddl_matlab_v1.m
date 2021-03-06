    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
close all
% rosshutdown
% rosinit
addpath('../rrr version phil v2')
addpath('../rrr version phil/geom3d/geom3d')
addpath('../rrr version phil/geom3d/meshes3d')
addpath('../rrr version phil/p_poly_dist_v1')
addpath('../rrr version phil/joystick soft 2')
addpath('../rrr version phil/Remesher')

link_2_gazebo = false;
c = computer;
if strcmp(c, 'PCWIN64')
    comp_windows = 1;
    my_joystick = vrjoystick(2);
else
    comp_windows = 0;
    my_joystick = vrjoystick(1);
end
% contact_subscriber = rossubscriber('/forces');
% contacts = receive(contact_subscriber,10);
% for i = 1:100
%     pose_data = receive(robot_joint_subscriber,10);
%     pose_data.Actual.Positions
% end
% rosshutdown
% % Mouse3D('start');
dh_eff = dh_UR5();
my_joystick = vrjoystick(1);
last_cond = 0;

% trouver l'état actuel du robot

Robot_Pose_j = [-1,-2,2,0,1,0];

pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)];
pose_init;
% on examine l'état des collisions au départ de l'algo

jacob_eff = jacob_UR5(Robot_Pose_j ,pose_init, dh_eff);
%afficherLimites(limit);
theta_dot_threshold = 0.0001;
min_distance = 20;
membrures_robot = get_membrures_robot();
limit = StructureLimites_v6();

%affichage des membrures positionn�es.
fig=figure(1);
set(fig, 'renderer', 'OpenGL');
camlight;
axis vis3d equal;
view(2);
%set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
xlabel('x')
ylabel('y')
zlabel('z')
axis([-3 3 -3 3 -0.5 3]);
afficherLimites(limit);
mat_ligne_x=[1 2];
mat_ligne_y=[1 2];
mat_ligne_z=[1 2];
h_robot_lines = line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','b');
h_collision_lines = line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');


if link_2_gazebo
    [robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, ~] = ur5_ros_controller_init();
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
end

while 1
    tic;
    Robot_Poses = cin_dir_6ddl(wrapToPi(Robot_Pose_j), dh_eff);
    Robot_Poses = Robot_Poses(1:3,4)';
    
    [ dir, rot ] = read_joystick_inputs( my_joystick, comp_windows);
    
    % On cherche si un objet entre en collision avec le robot
    %[normale_effecteur, collision_pose_eff, d_min, collision_poses, membrures_colisions] = collision_manager(contact_subscriber, Robot_Pose_j, dh_eff, membrures_robot);
    [normale_effecteur_matlab, d_min_matlab, pose_prox, poses_prox_pt_act,h_collision_lines] = collision_manager_matlab(limit, Robot_Pose_j, dh_eff, h_collision_lines, min_distance);
    
    %on garde en memoire l'input de l'utilisateur
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    v_prem = [dir rot];
    normale_effecteur_matlab = check_redund_v3(Robot_Poses(1,:), normale_effecteur_matlab,d_min_matlab, pose_prox);
    v_input = verifVitesse_trans_rot_v0(v_prem,normale_effecteur_matlab,d_min_matlab);
    % v_input=verifVitesse_v9(v_prem, normale_effecteur_matlab, d_min_matlab, min_distance);
%     if ~isempty(normale_effecteur)
%         dir=verifVitesse_v7(dir,normale_effecteur(:,1:3),d_min, min_gap * 1);
%         rot=verifVitesse_v7(rot,-normale_effecteur(:,4:6),d_min, min_gap * 1);
%     end
    [ next_ang, theta_dot, next_pose ] = move_ur5_robot_v3(v_input, last_cond, dh_eff, Robot_Pose_j);
    
    theta_dot = SpeedLimiter(theta_dot, 1);
    
    
    if link_2_gazebo
        [ Robot_Pose_j_history ] = ros_ur_controller_manager_v1( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history);
    end
    
    %affichage du robot dans le graphique matlab
    [h_robot_lines] = display_robot_UR5(Robot_Pose_j, dh_eff, h_robot_lines);
    
    Robot_Pose_j = Robot_Pose_j + theta_dot * 0.01;
    
    drawnow limitrate
    time = toc;
    while time < 0.01
        time = toc;
    end
end





















