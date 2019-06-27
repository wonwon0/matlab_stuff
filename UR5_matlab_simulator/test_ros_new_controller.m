    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
% rosshutdown
% rosinit
% dh=dh_UR5(6, 1, 180);
% armJoints = [-1,-1,-1,-1,-1,-1];
% theta_dot=[0;0;0;0;0;0];
% robot_joint_subscriber = rossubscriber('/ur/joint_state');
% pose_data = receive(robot_joint_subscriber,10);
% [joint_cmd_publisher, joint_cmd_message] = rospublisher('/ur/joint_control', 'sensor_msgs/JointState');
% % pause(2)
% % joint_cmd_msg = rosmessage(joint_cmd_publisher);
% % duration = robotics.ros.msg.Duration;
% % duration.Sec = 5;
% joint_cmd_message.Name = pose_data.Name;
% joint_cmd_message.Position = [0, 2,-2,1,0,1,0,0,0,0]';
% send(joint_cmd_publisher,joint_cmd_message)
% pause(5)

[robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, dh] = ur5_ros_controller_init();
my_joystick = vrjoystick(1);
last_cond = 0;
pose_data = receive(robot_joint_subscriber,10);
Robot_Pose_j = pose_data.Position(2:7)';
last_pose=cin_dir_6ddl(Robot_Pose_j,dh);
last_pose_ang = Robot_Pose_j;
Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
theta_dot_threshold = 0.0001;

while 1
    tic;
    
    [ dir, rot ] = read_joystick_inputs( my_joystick );
    
    [ next_ang, theta_dot, next_pose ] = move_ur5_robot( dir, rot, last_cond, robot_joint_subscriber, dh );
    
    [ Robot_Pose_j_history ] = ros_ur_controller_manager( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history);
    
    
    last_pose = next_pose;
    last_pose_ang = next_ang;
end





















