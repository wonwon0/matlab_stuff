function [ Robot_Pose_j_history ] = ros_ur_controller_manager_v1( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history)
    theta_dot_mod = [0, theta_dot, 0, 0, 0];
    next_ang_mod = [0 next_ang 0 0 0];
    joints_name = pose_data.Name;
    
    joint_cmd_message.Name = joints_name;
    joint_cmd_message.Position = next_ang_mod;
    joint_cmd_message.Velocity = next_ang_mod * 0;
    
    send(joint_cmd_publisher,joint_cmd_message);




end

