    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
% rosshutdown
% rosinit
addpath('../rrr version phil v2')
addpath('../rrr version phil/geom3d/geom3d')
addpath('../rrr version phil/geom3d/meshes3d')
addpath('../rrr version phil/p_poly_dist_v1')
addpath('../rrr version phil/joystick soft 2')
addpath('../rrr version phil/Remesher')
[robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, dh] = ur5_ros_controller_init();
contact_subscriber = rossubscriber('/forces');
contacts = receive(contact_subscriber,10);
dh_eff = dh;
% for i = 1:100
%     pose_data = receive(robot_joint_subscriber,10);
%     pose_data.Actual.Positions
% end
% rosshutdown
% % Mouse3D('start');

my_joystick = vrjoystick(1);
last_cond = 0;

% trouver l'état actuel du robot
pose_data = receive(robot_joint_subscriber,10);
Robot_Pose_j = pose_data.Position(2:7)';
Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)];
pose_init;
% on examine l'état des collisions au départ de l'algo

jacob_eff = jacob_UR5(Robot_Pose_j ,pose_init, dh(1));
%afficherLimites(limit);
theta_dot_threshold = 0.0001;
min_gap = 1;
membrures_robot = get_membrures_robot();
limit = StructureLimites_v3();

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
h8=line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');
while 1
    tic;
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = pose_data.Position(2:7)';
    Robot_Poses = cin_dir_6ddl(wrapToPi(Robot_Pose_j), dh);
    Robot_Poses = Robot_Poses(1:3,4)';
    
    [ dir, rot ] = read_joystick_inputs( my_joystick );
    % On cherche si un objet entre en collision avec le robot
    [normale_effecteur, collision_pose_eff, d_min, collision_poses, membrures_colisions] = collision_manager(contact_subscriber, Robot_Pose_j, dh_eff, membrures_robot);
    [normale_effecteur_matlab, d_min_matlab, pose_prox, poses_prox_pt_act,h8] = collision_manager_matlab(limit, Robot_Pose_j, dh_eff, h8);
    %on garde en memoire l'input de l'utilisateur
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    v_prem = [dir rot];
    % normale_effecteur = check_redund_v3(Robot_Poses(1,:), normale_effecteur,d_min, collision_pose_eff);
    v_input=verifVitesse_v8(v_prem,normale_effecteur,d_min, min_gap * 1);
%     if ~isempty(normale_effecteur)
%         dir=verifVitesse_v7(dir,normale_effecteur(:,1:3),d_min, min_gap * 1);
%         rot=verifVitesse_v7(rot,-normale_effecteur(:,4:6),d_min, min_gap * 1);
%     end
    [ next_ang, theta_dot, next_pose ] = move_ur5_robot_v2(v_input, last_cond, dh, Robot_Pose_j);
    
    theta_dot = SpeedLimiter(theta_dot, 1);
    
    [ Robot_Pose_j_history ] = ros_ur_controller_manager( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history);
    % Robot_Pose_j = next_ang;
    Robot_Pose_j = Robot_Pose_j_history;
    
    drawnow limitrate
end





















