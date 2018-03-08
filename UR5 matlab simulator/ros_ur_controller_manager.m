function [ Robot_Pose_j_history ] = ros_ur_controller_manager( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history)
    theta_dot_mod = [0, theta_dot, 0, 0, 0];
    next_ang_mod = [0 next_ang 0 0 0];
    stationnary_joints = abs(theta_dot_mod)-theta_dot_threshold < 0;
    name_stationnairy = pose_data.Name(stationnary_joints);
    pose_stationnary = Robot_Pose_j_history(stationnary_joints);
    pose_moving = Robot_Pose_j_history(not(stationnary_joints));
    
    name_moving = pose_data.Name(not(stationnary_joints));
    theta_dot_moving = theta_dot_mod(not(stationnary_joints));
    
    joint_cmd_message.Name = name_stationnairy;
    joint_cmd_message.Position = pose_stationnary;
    joint_cmd_message.Velocity = pose_stationnary * 0;
    
    send(joint_cmd_publisher,joint_cmd_message);
    if not(all(stationnary_joints))
        joint_cmd_message.Name = name_moving;
        joint_cmd_message.Position = next_ang_mod;
        joint_cmd_message.Velocity = theta_dot_moving;
        send(joint_cmd_publisher,joint_cmd_message)
        Robot_Pose_j_history(not(stationnary_joints)) = next_ang_mod(not(stationnary_joints));
    end
    time=toc;
    while time <= 0.01
        tic;
        pause(0.001)
        time = time + toc;
    end


end

