    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
rosshutdown
rosinit
dh=dh_UR5(6, 1, 180);
armJoints = [-1,-1,-1,-1,-1,-1];
theta_dot=[0;0;0;0;0;0];
robot_joint_subscriber = rossubscriber('/arm_controller/state');
pose_data = receive(robot_joint_subscriber,10);
joint_cmd_publisher = rospublisher('/arm_controller/command', 'trajectory_msgs/JointTrajectory');
pause(2)
joint_cmd_msg = rosmessage(joint_cmd_publisher);
duration = robotics.ros.msg.Duration;
duration.Sec = 5;
joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
joint_cmd_msg.JointNames = pose_data.JointNames;
joint_send.Positions = [1,-1,1,0,1,0];
%joint_send.Accelerations = ones(1,6);
joint_send.Velocities = [0;0;0;0;0;0];
joint_send.TimeFromStart = duration;
joint_send.Effort = ones(1,6)*20000;
joint_cmd_msg.Points = joint_send;
send(joint_cmd_publisher,joint_cmd_msg)
pause(5)
duration.Nsec = 40000000;
% for i = 1:100
%     pose_data = receive(robot_joint_subscriber,10);
%     pose_data.Actual.Positions
% end
% rosshutdown
% % Mouse3D('start');
my_joystick = vrjoystick(1);
last_cond = 0;
Robot_Pose_j = pose_data.Actual.Positions;
last_pose=cin_dir_6ddl(Robot_Pose_j,dh);
last_pose_ang = Robot_Pose_j;
while 1
    tic;
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = wrapToPi(pose_data.Actual.Positions);
    Robot_Pose=cin_dir_6ddl(Robot_Pose_j,dh);
    %(unitaire)     v_x                 v_y                    v_z
    a = [axis(my_joystick,5), axis(my_joystick,6), (button(my_joystick,7) - button(my_joystick,8))];
    
    %(unitaire)     w_x                  w_y                 w_z
    b = [axis(my_joystick,1), axis(my_joystick,2), axis(my_joystick,3)];
    if norm(b) > 0
        b = b/norm(b) / 1000;
    end
%     examples_tous_les_boutons = [axis(my_joystick,1), axis(my_joystick,2), axis(my_joystick,3), ...
%         axis(my_joystick,4), axis(my_joystick,5), axis(my_joystick,6), ...
%         button(my_joystick,1), button(my_joystick,2), button(my_joystick,3),...
%         button(my_joystick,4), button(my_joystick,5), button(my_joystick,6),...
%         button(my_joystick,7), button(my_joystick,8), button(my_joystick,9)]
    
    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[double(a(2)),double(a(1)),-double(a(3))];
    else
        dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
    dir = dir;
    current_rotation_mat = Robot_Pose(1:3, 1:3);

    next_rotation_mat = current_rotation_mat * eul2rotm(b, 'ZYX');
    next_pose = [next_rotation_mat(1,:) dir(1) + Robot_Pose(1,4);...
                 next_rotation_mat(2,:) dir(2) + Robot_Pose(2,4);...
                 next_rotation_mat(3,:) dir(3) + Robot_Pose(3,4);...
                 0 0 0 1]*1;
    next_ang = cin_inv_6ddl(next_pose,dh,Robot_Pose_j);
    theta_dot = next_ang - Robot_Pose_j;

    
%     for i =1:6
%         if (next_ang(i) - Robot_Pose_j(i))>pi
%             theta_dot(i)=-next_ang(i) - Robot_Pose_j(i)-2*pi;
%         elseif (next_ang(i) - Robot_Pose_j(i))<-pi
%             theta_dot(i)=next_ang(i) - Robot_Pose_j(i)+2*pi;
%         else
%             theta_dot(i)=next_ang(i) - Robot_Pose_j(i);
%         end
%     end
    jac=jacob_UR5(next_ang,Robot_Pose,dh);
    cond_jac = cond(jac)
    if cond_jac>5000
        if last_cond < cond_jac
            theta_dot = theta_dot * 0;
        end
            
        last_cond = cond_jac;
    end
    
    theta_dot=theta_dot / (duration.Nsec / 10^9);
    %theta_dot = SpeedLimiter(theta_dot, 0.5);
    theta_dot;
    joint_send.Positions = next_ang';
    joint_send.Velocities = theta_dot';
    joint_cmd_msg.Points = joint_send;
    send(joint_cmd_publisher,joint_cmd_msg)
    time=toc;
    while time <= (duration.Nsec / 10^9)
        tic;
        pause(0.01)
        time = time + toc;
    end
    last_pose = next_pose;
    last_pose_ang = next_ang;
    time;
end





















