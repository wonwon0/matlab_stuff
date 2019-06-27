function [ jac ] = jacob_UR5( theta_act, Pose, dh, membrure)
    %theta_act: position angulaire des joint du robot
    %pose: position et orientation de l'effecteur
    %dh: structure contenant les param. dh du robot
    if nargin < 4
        membrure = 6;
    end
    if length(Pose)>3
        lol = 1;
    end
    theta=theta_act;
    x=Pose(1);y=Pose(2);z=Pose(3);
    s = sin(theta_act);
    c = cos(theta_act);
    
    jac=[ 0 s(1) s(1) s(1) c(1)*sin(theta(2)+theta(3)+theta(4)) -c(1)*cos(theta(2)+theta(3)+theta(4))*s(5)+s(1)*c(5);
        0 -c(1) -c(1) -c(1) s(1)*sin(theta(2)+theta(3)+theta(4)) -s(1)*cos(theta(2)+theta(3)+theta(4))*s(5)-c(1)*c(5);
        1 0 0 0 -cos(theta(2)+theta(3)+theta(4)) -sin(theta(2)+theta(3)+theta(4))*s(5);
        -y -c(1)*(z-dh.b(1)) -c(1)*(z-dh.b(1)-dh.a(2)*s(2)) c(1)*(dh.b(5)*cos(theta(2)+theta(3)+theta(4))+dh.b(6)*sin(theta(2)+theta(3)+theta(4))*s(5)) -dh.b(6)*(s(1)*s(5)+c(1)*cos(theta(2)+theta(3)+theta(4))*c(5)) 0;
        x -s(1)*(z-dh.b(1)) -s(1)*(z-dh.b(1)-dh.a(2)*s(2)) s(1)*(dh.b(5)*cos(theta(2)+theta(3)+theta(4))+dh.b(6)*sin(theta(2)+theta(3)+theta(4))*s(5)) dh.b(6)*(c(1)*s(5)-s(1)*cos(theta(2)+theta(3)+theta(4))*c(5)) 0;
        0 c(1)*x+s(1)*y dh.a(3)*cos(theta(2)+theta(3))+dh.b(5)*sin(theta(2)+theta(3)+theta(4))-dh.b(6)*cos(theta(2)+theta(3)+theta(4))*s(5) dh.b(5)*sin(theta(2)+theta(3)+theta(4))-dh.b(6)*cos(theta(2)+theta(3)+theta(4))*s(5) -dh.b(6)*sin(theta(2)+theta(3)+theta(4))*c(5) 0];
end

