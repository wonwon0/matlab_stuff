    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
% rosshutdown
% rosinit
addpath('../rrr version phil v2')
dh_eff=dh_UR5();
dh = [dh_eff]
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

% trouver l'état actuel du robot
Robot_Pose_j = pose_data.Actual.Positions;
pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)]
% on examine l'état des collisions au départ de l'algo

jacob_eff = jacob_UR5(Robot_Pose_j ,pose_init, dh(1));
[d_min, pose_prox, pose_prox_pt_act] = verifDistance_v5(limit, pose_init, jacob_eff, jacob_eff, Robot_Pose_j);
%afficherLimites(limit);

while 1
    tic;
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = pose_data.Actual.Positions;
    Robot_Poses = [];
    for i = 1:length(dh)
        robot_pose = cin_dir_6ddl(Robot_Pose_j,dh(i));
        Robot_Poses = [Robot_Poses; [robot_pose(1,4), robot_pose(2,4), robot_pose(3,4)]];
    end
    a = [axis(my_joystick,1), axis(my_joystick,2), axis(my_joystick,3)];

    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[double(a(2)),double(a(1)),-double(a(3))];
    else
        dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
    dir = dir * 10;
    % On cherche si un objet entre en collision avec le robot
    pose_prox=[];d_min=[];
    jacob_eff = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh(1));
    for i=1:size(Robot_Poses,1)
        jacob_local = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh(i));
        [d_min_t,pose_prox_t,pose_prox_pt_act]=verifDistance_v5(limit,Robot_Poses(i, :), jacob_local,jacob_eff,Robot_Pose_j);
        d_min=[d_min d_min_t];
        pose_prox=[pose_prox; pose_prox_pt_act];
        poses_prox_pt_act.pose(i).poses=pose_prox_pt_act;
    end
    %on garde en m�moire l'input de l'utilisateur
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    normale_effecteur=[];
    d_min_relevant=[];
    for i = 1:2:length(dh)
        jacob_pt=jacob_UR5(Robot_Pose_j ,Robot_Poses(i, :), dh(i));
        m=size(limit.limite,2);
        for j=1:m
            if d_min((i-1)*m+j)<0.02 && ((all(limit.limite(j).type=='tube'))||(all(limit.limite(j).type=='sphe')))
                if (all(limit.limite(j).type=='tube')) && ddl(i)==2
                    ;
                else
                    normale_effecteur_loc=PointToEffector(pose_prox((i-1)*m+j,:), Robot_Poses(i,:), jacob_pt, jacob_eff );
                    normale_effecteur=[normale_effecteur;normale_effecteur_loc];
                    d_min_relevant=[d_min_relevant;d_min((i-1)*m+j)];
                end
            elseif d_min((i-1)*m+j)<0.02
                normale_effecteur_loc=PointToEffector(pose_prox((i-1)*m+j,:), Robot_Poses(i,:), jacob_pt, jacob_eff );
                normale_effecteur=[normale_effecteur;normale_effecteur_loc];
                d_min_relevant=[d_min_relevant;d_min((i-1)*m+j)];
            end
        end
    end
    %%%%%%%%%%
    v_prem = dir;
    normale_effecteur = check_redund_v2(Robot_Poses(1,:), normale_effecteur,d_min_relevant,pose_prox);
    v_input=verifVitesse_v7(v_prem,normale_effecteur,d_min_relevant,0.01);
%     if slide==0
%         if ~all(v_prem==v_input)
%             v_input=[0 0 0];
%         end
%     end
    robot_pose = cin_dir_6ddl(Robot_Pose_j,dh(1));
    next_pose=robot_pose+[0 0 0 v_input(1);0 0 0 v_input(2);0 0 0 v_input(3); 0 0 0 0]*1;
    next_ang=cin_inv_6ddl(next_pose,dh_eff,Robot_Pose_j);
    for i =1:6
        if (next_ang(i) - Robot_Pose_j(i))>pi
            theta_dot(i)=-next_ang(i) - Robot_Pose_j(i)-2*pi;
        elseif (next_ang(i) - Robot_Pose_j(i))<-pi
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i)+2*pi;
        else
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i);
        end
    end
    jac=jacob_UR5(next_ang,Robot_Poses(1, :),dh(1));
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
    d_min
end





















