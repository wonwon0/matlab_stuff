    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
rosshutdown
rosinit
dh=dh_UR5();
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
joint_send.Positions = [0,-1,1,0,1,0];
%joint_send.Accelerations = ones(1,6);
%joint_send.Velocities = [1,1,1,1,1,1];
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

% load limit bodies
limit=StructureLimites_v2();

% on examine l'état des collisions au départ de l'algo

[d_min, pose_prox, pose_prox_pt_act]=verifDistance_v3(limit,pose_init,L(3,:),L(3,:),theta_act);



while 1
    tic;
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = pose_data.Actual.Positions;
    Robot_Pose=cin_dir_6ddl(Robot_Pose_j,dh);

    a = [axis(my_joystick,1), axis(my_joystick,2), axis(my_joystick,3)];

    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[double(a(2)),double(a(1)),-double(a(3))];
    else
        dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
    dir = dir * 10;
    next_pose=Robot_Pose+[0 0 0 dir(1);0 0 0 dir(2);0 0 0 dir(3); 0 0 0 0]*1;
    next_ang=cin_inv_6ddl(next_pose,dh,Robot_Pose_j);
    for i =1:6
        if (next_ang(i) - Robot_Pose_j(i))>pi
            theta_dot(i)=-next_ang(i) - Robot_Pose_j(i)-2*pi;
        elseif (next_ang(i) - Robot_Pose_j(i))<-pi
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i)+2*pi;
        else
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i);
        end
    end
    jac=jacob_UR5(next_ang,Robot_Pose,dh);
    cond_jac = cond(jac);
    if cond_jac>5000
        if last_cond < cond_jac
            theta_dot = theta_dot * 0;
        end
            
        last_cond = cond_jac;
    end
    
    theta_dot=theta_dot / (duration.Nsec / 10^9);
    theta_dot = SpeedLimiter(theta_dot, 1);
    theta_dot;
    joint_send.Positions = next_ang';
    joint_send.Velocities = theta_dot';
    joint_cmd_msg.Points = joint_send;
    send(joint_cmd_publisher,joint_cmd_msg)
    time=toc;
    while time <= (duration.Nsec / 10^9)
        tic;
        pause(0.001)
        time = time + toc;
    end
    time;
end





















