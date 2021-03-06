function [ pose_act ] = cin_dir_6ddl( theta_act, L )
    %cin_dir r�soud le probl�me g�om�trique directe du robot RRR
    theta1=theta_act(1)+pi/2;theta2=theta_act(2)+pi/2;theta3=theta_act(3);

    for i=1:size(L,1)
        x=L(i,2)*cos(theta1)*cos(theta2)+L(i,3)*cos(theta1)*cos(theta2)*cos(theta3)-...
            L(i,3)*cos(theta1)*sin(theta2)*sin(theta3)-L(i,4)*sin(theta1);
        y=L(i,2)*sin(theta1)*cos(theta2)+L(i,3)*sin(theta1)*cos(theta2)*cos(theta3)-...
            L(i,3)*sin(theta1)*sin(theta2)*sin(theta3)+L(i,4)*cos(theta1);
        z=L(i,1)+L(i,2)*sin(theta2)+L(i,3)*sin(theta2)*cos(theta3)+L(i,3)*cos(theta2)*sin(theta3);
        pose_act(i,:)=[x y z];
    end
end

