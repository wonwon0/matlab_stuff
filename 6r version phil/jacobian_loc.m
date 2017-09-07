function [jacob]=jacobian_loc(theta, L, ddl)
    theta=theta+[pi/2 pi/2 0];
    
    if ddl==3
        %theta est un vecteur rangée de longueur 3
        jacob=[-sin(theta(1))*cos(theta(2))*L(2)-sin(theta(1))*cos(theta(2))*cos(theta(3))*L(3)+sin(theta(1))*sin(theta(2))*sin(theta(3))*L(3)-cos(theta(1))*L(4) -cos(theta(1))*(sin(theta(2))*L(2)+sin(theta(2))*cos(theta(3))*L(3)+cos(theta(2))*sin(theta(3))*L(3)) -cos(theta(1))*L(3)*(sin(theta(2))*cos(theta(3))+cos(theta(2))*sin(theta(3)))
            cos(theta(1))*cos(theta(2))*L(2)+cos(theta(1))*cos(theta(2))*cos(theta(3))*L(3)-cos(theta(1))*sin(theta(2))*sin(theta(3))*L(3)-sin(theta(1))*L(4) -sin(theta(1))*(sin(theta(2))*L(2)+sin(theta(2))*cos(theta(3))*L(3)+cos(theta(2))*sin(theta(3))*L(3)) -sin(theta(1))*L(3)*(sin(theta(2))*cos(theta(3))+cos(theta(2))*sin(theta(3)))
            0 cos(theta(2))*cos(theta(3))*L(3)-sin(theta(2))*sin(theta(3))*L(3)+cos(theta(2))*L(2) L(3)*(cos(theta(2))*cos(theta(3))-sin(theta(2))*sin(theta(3)))];
    elseif ddl==2
        jacob=[-sin(theta(1))*cos(theta(2))*L(2) -sin(theta(2))*cos(theta(1))*L(2);
            cos(theta(2))*cos(theta(1))*L(2) -sin(theta(1))*sin(theta(2))*L(2);
            0 L(2)*cos(theta(2))];
    end
end