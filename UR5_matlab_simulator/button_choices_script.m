% button_choices_script
% fait la selection des option de boutton dans le main script de simulation

if (other_buttons(10) == 1) && link_2_gazebo
    for i = 1:length(test_mode)
        disp('*************************RESET***************************')
        if test_mode(i)==0
            test_mode(i) = interactive_test_solids(Robot_Poses, i * 10);
        end
    end
    Robot_Pose_j = [-1,-2,2,0,1,0];

    pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
    pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)];
    pose_init;
    % on examine l'état des collisions au départ de l'algo

    jacob_eff = jacob_UR5_v1(Robot_Pose_j ,pose_init, dh_eff);
    ur5_ros_controller_reset(joint_cmd_publisher, joint_cmd_message, pose_data, Robot_Pose_j);
    Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
end

if other_buttons(1) == 1
    disp('**********************SLIDE_MODE = 1 ************************')
    disp('**********************SLIDE_MODE = 1 ************************')
    disp('**********************SLIDE_MODE = 1 ************************')
    sliding_mode = 1;
elseif other_buttons(2) == 1
    disp('**********************SLIDE_MODE = 2 ************************')
    disp('**********************SLIDE_MODE = 2 ************************')
    disp('**********************SLIDE_MODE = 2 ************************')
    sliding_mode = 2;
elseif other_buttons(3) == 1
    disp('**********************SLIDE_MODE = 3 ************************')
    disp('**********************SLIDE_MODE = 3 ************************')
    disp('**********************SLIDE_MODE = 3 ************************')
    sliding_mode = 3;
elseif other_buttons(4) == 1
    disp('**********************SLIDE_MODE = 4 ************************')
    disp('**********************SLIDE_MODE = 4 ************************')
    disp('**********************SLIDE_MODE = 4 ************************')
    sliding_mode = 4;
end