    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear all

% Connect to robot
Robot_IP = '132.203.102.123';
Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server');
fclose(Socket_conn);
disp('Press Play on Robot...')
fopen(Socket_conn);
disp('Connected!');
dh=dh_UR5();
disp('connection VREP...');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
if (id>-1)
    disp('Connected to VREP remote API server');
end
h = struct('id', id);
armJoints = [-1,-1,-1,-1,-1,-1];
theta_dot=[0;0;0;0;0;0];
for i = 1:6
  [res armJoints(i)] = vrep.simxGetObjectHandle(id, sprintf('UR5_joint%d',i), vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
end
h.armJoints=armJoints;
Robot_Pose_j = readrobotpose_j(Socket_conn);
for i = 1:6
    res = vrep.simxSetJointPosition(id, h.armJoints(i),...
                                         Robot_Pose_j(i),...
                                 vrep.simx_opmode_oneshot);
end
% Mouse3D('start');
while 1
    tic;
%     Robot_Pose = readrobotpose(Socket_conn);
%     Robot_Pose=Robot_Pose';
    Robot_Pose_j = readrobotpose_j(Socket_conn);
    Robot_Pose_j=Robot_Pose_j';
    Robot_Pose=cin_dir_6ddl(Robot_Pose_j,dh);
    Robot_Pose_vrep=Robot_Pose_j-dh.theta;
    for i = 1:6
    res = vrep.simxSetJointPosition(id, h.armJoints(i),...
                                         Robot_Pose_vrep(i),...
                                    vrep.simx_opmode_oneshot);
    end
    %communication avec la souris 3D (pause(0.01) est nécessaire pour pas
    %overflow le capteur)
%     pause(0.01)
%     input3D = Mouse3D('get');
%     a=input3D.pos;
%     a=[-a(3) -a(1) -a(2)];
    %communication avec le joystick
    [pos, but] = mat_joy(1);
    a = pos;
    if but(2)==1
        ligne=0;
    end
    if but(3)==1
        ligne=1;
    end
    %Saturation de la commande de vitesse (s'assurer que la norme soit
    %moins grande que 1)
    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[double(a(2)),double(a(1)),-double(a(3))];
    else
        dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
%     jac=jacob_UR5(Robot_Pose_j,Robot_Pose,dh);
    
%     theta_dot=[a(1) a(2) a(3) 0 0 0]/2;
     next_pose=Robot_Pose+[0 0 0 dir(1);0 0 0 dir(2);0 0 0 dir(3); 0 0 0 0]*1;
%     next_pose=Robot_Pose;
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
    time=toc;
    jac=jacob_UR5(next_ang,Robot_Pose,dh);
    if cond(jac)>6000
        next_ang(5)
    end
    
    theta_dot=theta_dot/time;
%     Translation = Robot_Pose(1:3); % in mm
%     Orientation = Robot_Pose(4:6);
%     Translation = Translation + [a(1) a(2) a(3)]'*20;
    speedrobot(Socket_conn,theta_dot');

end





















