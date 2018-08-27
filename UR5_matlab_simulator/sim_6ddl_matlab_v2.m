    

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
addpath('./rrr version phil/p_poly_dist_v1')
addpath('../rrr version phil/joystick soft 2')
addpath('../rrr version phil/Remesher')

% chargement parametres externes (test_mode, link_2_gazebo, auto_mode)
param_script_sim
% contact_subscriber = rossubscriber('/forces');
% contacts = receive(contact_subscriber,10);
% for i = 1:100
%     pose_data = receive(robot_joint_subscriber,10);
%     pose_data.Actual.Positions
% end
% rosshutdown
% % Mouse3D('start');
dh_eff = dh_UR5_v1(110);
c = computer;
if strcmp(c, 'PCWIN64')
    comp_windows = 1;
    my_joystick = vrjoystick(2);
else
    comp_windows = 0;
    my_joystick = vrjoystick(1);
end

last_cond = 0;

% trouver l'état actuel du robot

Robot_Pose_j = [-1,-2,2,0,1,0];

pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)];
pose_init;
% on examine l'état des collisions au départ de l'algo

jacob_eff = jacob_UR5_v1(Robot_Pose_j ,pose_init, dh_eff);
%afficherLimites(limit);
theta_dot_threshold = 0.0001;
min_distance = 50;
singularite_tol = 0.4;
membrures_robot = get_membrures_robot();
limit = StructureLimites_v6();

%affichage des membrures positionn�es.
fig=figure(1);
set(fig, 'renderer', 'OpenGL');
camlight;
axis vis3d equal;
view(135, 45);
%set(gca,'Ydir','reverse')
%set(gca,'Xdir','reverse')

axis([-3 3 -3 3 -0.5 3]);
afficherLimites(limit);
mat_ligne_x=[1 2];
mat_ligne_y=[1 2];
mat_ligne_z=[1 2];
h_robot_lines = line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',5,'color','b');
h_collision_lines = line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');

xlabel('x')
ylabel('y')
zlabel('z')
if link_2_gazebo
    write_limits('test_poly', limit)
    pause(1)
    create_world_file('test_mass_spawn/')
    pause(1)
    if auto_launch
        [status,cmdout] = system(['export LD_LIBRARY_PATH="/home/phil/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";' 'roslaunch ur_on_table ur_on_table_gazebo_controlled.launch & echo $!']);
        % permet la fermeture de gazebo et de ros quand la fonction
        % launch_sim() est utilisée pour lancer le script et qu'un erreur
        % ou un interruption utilisateur survient. 
        finishup = onCleanup(@() myCleanupFun(cmdout));
        pause(10)
    end
    [robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, ~] = ur5_ros_controller_init();
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
    gazebo = ExampleHelperGazeboCommunicator();
end
normales_effecteur_matlab = [];
normales_effecteur_singularite = [];
while 1
    tic;

    Robot_Poses = cin_dir_6ddl(wrapToPi(Robot_Pose_j), dh_eff);
    Robot_Poses = Robot_Poses(1:3,4)';
    if test_mode(1) && auto_launch
        test_mode(1) = interactive_test_solids(Robot_Poses, 1);
    end
    if test_mode(2) && auto_launch
        test_mode(2) = interactive_test_solids(Robot_Poses, 2);
    end
    if test_mode(3) && auto_launch
        test_mode(3) = interactive_test_solids(Robot_Poses, 3);
    end
    
    [ dir, rot, other_buttons ] = read_joystick_inputs( my_joystick, comp_windows);
    
    button_choices_script
    
    % On cherche si un objet entre en collision avec le robot
    %[normale_effecteur, collision_pose_eff, d_min, collision_poses, membrures_colisions] = collision_manager(contact_subscriber, Robot_Pose_j, dh_eff, membrures_robot);
    [normales_effecteur_matlab, d_min_matlab, pose_prox, h_collision_lines, poses_articulations] = collision_manager_matlab_v1(limit, Robot_Pose_j, dh_eff, h_collision_lines, min_distance, link_2_gazebo);
    [normales_effecteur_singularite, d_min_singularites] = singularities_UR5_finder_V1(Robot_Pose_j, Robot_Poses, dh_eff, singularite_tol)
    normales_effecteur= [normales_effecteur_matlab; normales_effecteur_singularite];
    d_min = [d_min_matlab min_distance*d_min_singularites];
    %on garde en memoire l'input de l'utilisateur
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    v_prem = [dir rot];
    %normale_effecteur = check_redund_v3(Robot_Poses(1,:), normale_effecteur_matlab,d_min_matlab, pose_prox);
    if sliding_mode == 1
        v_input = verifVitesse_trans_rot_v0(v_prem,normales_effecteur,d_min);
    elseif sliding_mode == 2
        v_input=verifVitesse_v9(v_prem, normales_effecteur, d_min, min_distance);
    elseif sliding_mode == 3
        v_input=verifVitesse_ForceField(v_prem,normales_effecteur,d_min, min_distance);
    else
        v_input=verifVitesse_v9(v_prem, normales_effecteur, d_min, min_distance);
        if any(v_prem ~= v_input)
            v_input = v_input * 0;
        end
    end
    [ next_ang, theta_dot, next_pose ] = move_ur5_robot_v4(v_input, last_cond, dh_eff, Robot_Pose_j);
    
    theta_dot = SpeedLimiter(theta_dot, 1);
    next_ang = next_ang + theta_dot * 0.015;
    
    
    if link_2_gazebo
        [ Robot_Pose_j_history ] = ros_ur_controller_manager_v1( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history);
    
    else
        %affichage du robot dans le graphique matlab
        [h_robot_lines] = display_robot_UR5_v1(h_robot_lines, poses_articulations);
    end
    Robot_Pose_j = Robot_Pose_j + theta_dot * 0.015;
    
    drawnow limitrate
    time = toc;
    while time < 0.015
        time = toc;
    end
end























