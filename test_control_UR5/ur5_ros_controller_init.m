function [robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, dh] = ur5_ros_controller_init()
    rosshutdown
    rosinit
    [dh] = dh_UR5(6, 1, 180);
    robot_joint_subscriber = rossubscriber('/ur/joint_state');
    pose_data = receive(robot_joint_subscriber,10);
    [joint_cmd_publisher, joint_cmd_message] = rospublisher('/ur/joint_control', 'sensor_msgs/JointState');
    joint_cmd_message.Name = pose_data.Name;
    joint_cmd_message.Position = [0, 0,-2,2,0,1,0,0,0,0]';
    send(joint_cmd_publisher,joint_cmd_message)
pause(3)
end

