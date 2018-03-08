function [ next_ang, theta_dot, next_pose ] = move_ur5_robot_v2( v_input, last_cond, dh, Robot_Pose_j )
    dir = v_input(1:3);
    rot = v_input(4:6);
    
    [Robot_Pose, pose_euler] = cin_dir_6ddl_v2( Robot_Pose_j, dh);
    next_pose_euler = pose_euler + rot;

    next_rotation_mat = eul2rotm(next_pose_euler, 'ZYZ');
    next_pose = [next_rotation_mat(1,:) dir(1) + Robot_Pose(1,4);...
                 next_rotation_mat(2,:) dir(2) + Robot_Pose(2,4);...
                 next_rotation_mat(3,:) dir(3) + Robot_Pose(3,4);...
                 0 0 0 1]*1;
    next_ang = cin_inv_6ddl(next_pose,dh,Robot_Pose_j);
    theta_dot = (next_ang - Robot_Pose_j)/0.01;

    jac=jacob_UR5(next_ang,Robot_Pose,dh);
    cond_jac = cond(jac);
    if cond_jac>9000
        if last_cond < cond_jac
            theta_dot = theta_dot * 0;
            formatSpec = 'conditionnement %f';
            string = sprintf(formatSpec, cond_jac);
            display(string)
        else
            last_cond = cond_jac;
        end
            
        
    end
end

