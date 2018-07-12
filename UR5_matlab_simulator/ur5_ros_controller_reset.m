function ur5_ros_controller_reset(joint_cmd_publisher,joint_cmd_message, pose_data, joint_pose)
    joint_cmd_message.Name = pose_data.Name;
    joint_cmd_message.Position = [0, joint_pose,0,0,0]';
    send(joint_cmd_publisher,joint_cmd_message)
pause(3)
end

