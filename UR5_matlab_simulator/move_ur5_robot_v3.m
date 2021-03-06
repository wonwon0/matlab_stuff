function [ next_ang, theta_dot, next_pose ] = move_ur5_robot_v3( v_input, last_cond, dh, Robot_Pose_j )
    dir = v_input(1:3);
    rot = v_input(4:6);
    
    [Robot_Pose, ~] = cin_dir_6ddl_v2( Robot_Pose_j, dh);
    rot_mat = Robot_Pose(1:3, 1:3);

    next_rotation_mat =  eul2rotm(rot, 'XYZ') * rot_mat;
    next_pose = [next_rotation_mat(1,:) dir(1) + Robot_Pose(1,4);...
                 next_rotation_mat(2,:) dir(2) + Robot_Pose(2,4);...
                 next_rotation_mat(3,:) dir(3) + Robot_Pose(3,4);...
                 0 0 0 1]*1;
    next_ang = cin_inv_6ddl_v1(next_pose, Robot_Pose_j,dh);
    theta_dot = (next_ang - Robot_Pose_j)/0.01;
    
    jac=jacob_UR5(next_ang, next_pose(1:3,4), dh);
%     jac_test=jacob_UR5(Robot_Pose_j, Robot_Pose(1:3,4), dh);
%     theta_dot_verif = jac_test\ [rot dir]';
%     theta_dot/norm(theta_dot)
%     theta_dot_verif'/norm(theta_dot_verif)
    
    cond_jac = cond(jac);
    if cond_jac>9000
        if last_cond < cond_jac
            theta_dot = theta_dot * 0;
            next_ang = Robot_Pose_j;
            formatSpec = 'conditionnement %f';
            string = sprintf(formatSpec, cond_jac);
            display(string)
        else
            last_cond = cond_jac;
        end
            
        
    end
end

