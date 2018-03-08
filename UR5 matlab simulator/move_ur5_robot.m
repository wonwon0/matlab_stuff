function [ next_ang, theta_dot, next_pose ] = move_ur5_robot( dir, rot, last_cond, robot_joint_subscriber, dh )
    
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = pose_data.Position(2:7)';
    Robot_Pose_j = wrapToPi(Robot_Pose_j);
    Robot_Pose=cin_dir_6ddl(Robot_Pose_j,dh);
    current_rotation_mat = Robot_Pose(1:3, 1:3);

    next_rotation_mat = current_rotation_mat * eul2rotm(rot, 'ZYX');
    next_pose = [next_rotation_mat(1,:) dir(1) + Robot_Pose(1,4);...
                 next_rotation_mat(2,:) dir(2) + Robot_Pose(2,4);...
                 next_rotation_mat(3,:) dir(3) + Robot_Pose(3,4);...
                 0 0 0 1]*1;
    next_ang = cin_inv_6ddl(next_pose,dh,Robot_Pose_j);
    theta_dot = (next_ang - Robot_Pose_j)/0.01;

    jac=jacob_UR5(next_ang,Robot_Pose,dh);
    cond_jac = cond(jac);
    if cond_jac>8000
        if last_cond < cond_jac
            theta_dot = theta_dot * 0;
        end
            
        last_cond = cond_jac;
    end
end

