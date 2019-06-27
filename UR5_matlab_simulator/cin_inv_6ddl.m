function [ q_sols ] = cin_inv_6ddl( Pose,dh,theta_act)
%cin�matique inverse du robot UR5
%ce r�f�rer au m�moire de maitrise de Fran�ois L�VESQUES pour la th�orie
%"PRISE AUTONOME D�OBJETS DIVERS AVEC UN ROBOT S�RIEL INDUSTRIEL, UN PR�HENSEUR
% SOUS-ACTIONN� ET UNE CAM�RA 3D"
% Seuil au del� duquel la valeur est consid�r�e comme valant 0 (permet de r�duire l'impr�cision num�rique)
ZERO_THRESH = 10^-8;
% Q=[cos(euler(2))*cos(euler(1))*cos(euler(3))-sin(euler(1))*sin(euler(3)) -cos(euler(2))*cos(euler(1))*sin(euler(3))-sin(euler(1))*cos(euler(3)) sin(euler(2))*cos(euler(1));
%     cos(euler(2))*sin(euler(1))*cos(euler(3))+cos(euler(1))*sin(euler(3)) -cos(euler(2))*sin(euler(1))*sin(euler(3))+cos(euler(1))*cos(euler(3)) sin(euler(2))*sin(euler(1));
%     -sin(euler(2))*cos(euler(3)) sin(euler(2))*sin(euler(3)) cos(euler(2))];


% Nombre de solutions au probl�me g�om�trique inverse
        num_sols = 0;
        q_sols = [0 0 0 0 0 0]';
        %theta_act = theta_act + dh.theta';

%______________CALCUL DE THETA 1__________________
q1=[0,0];
A = dh.b(6)*Pose(2,3) - Pose(2,4);
B = dh.b(6)*Pose(1,3) - Pose(1,4);
R = A*A + B*B;
if abs(A) < ZERO_THRESH 
    if abs(abs(dh.b(4)) - abs(B)) < ZERO_THRESH 
        div = -sign(dh.b(4))*sign(B);
    else 
        div = -dh.b(4)/B;
    end

    arc_sin = asin(div);
    if abs(arc_sin) < ZERO_THRESH 
        arc_sin = 0.0;
    end
    if arc_sin < 0.0 
        q1(1) = arc_sin + 2.0*pi;
    else 
        q1(1) = arc_sin;
    end

    q1(2) = pi - arc_sin;

elseif abs(B) < ZERO_THRESH 
    if abs(abs(dh.b(4)) - abs(A)) < ZERO_THRESH 
        div = sign(dh.b(4))*sign(A);
    else 
        div = dh.b(4)/A;
    end
    arc_cos = acos(div);
    q1(1) = arc_cos;
    q1(2) = 2.0*pi - arc_cos;


elseif dh.b(4)*dh.b(4) > R 
    % S'il n'existe pas de solutions, quitter la fonction.
    q1(1)='nope';
    q1(2)='nope';

else 
    arc_cos = acos(dh.b(4) / sqrt(R));
    arc_tan = atan2(-B, A);
    pos = arc_cos + arc_tan;
    neg = -arc_cos + arc_tan;
    if abs(pos) < ZERO_THRESH
        pos = 0.0;
    end
    if abs(neg) < ZERO_THRESH 
        neg = 0.0;
    end
    if pos >= 0.0 
        q1(1) = pos;
    else 
        q1(1) = 2.0*pi + pos;
    end
    if neg >= 0.0 
        q1(2) = neg;
    else 
        q1(2) = 2.0*pi + neg;
    end
end
%S�LECTION DE theta1

for i=1:2
    if q1(i)>pi
        q1(i)=q1(i)-2*pi;
    elseif q1(i)<-pi
        q1(i)=q1(i)+2*pi;
    end
    if abs((q1(i)-theta_act(1))) < - pi
        diff_th1(i)=abs(abs(q1(i)-theta_act(1))+2*pi);
    elseif abs((q1(i)-theta_act(1))) > pi
        diff_th1(i)=abs(abs(q1(i)-theta_act(1))-2*pi);
    else
        diff_th1(i)=abs(q1(i)-theta_act(1));
    end
end
if diff_th1(1)>diff_th1(2)
    theta_1=q1(2);
else
    theta_1=q1(1);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if (sqrt(Pose(1)^2+Pose(2)^2)<0.001)
%     
%     
%     
%     theta1_1=theta_act(1);
%     theta1_2=theta_act(1);
% else
%     P_rep6=Pose(1:3)-[Q(1,3)*dh.b(6);Q(2,3)*dh.b(6);Q(3,3)*dh.b(6)];
%     gamma=atan2(P_rep6(2),P_rep6(1));
%     beta=acos(dh.b(4)/sqrt(P_rep6(1)^2+P_rep6(2)^2));
%     
%     theta1_1=gamma+beta+pi/2
%     theta1_2=gamma-beta+pi/2
% end

%______________CALCUL DE THETA 5__________________


numer = (Pose(1,4)*sin(theta_1) - Pose(2,4)*cos(theta_1)-dh.b(4));
if abs(abs(numer) - abs(dh.b(6))) < ZERO_THRESH 
    div = sign(numer) * sign(dh.b(6));
else 
    div = numer / dh.b(6);
end
arc_cos = acos(div);
if isreal(arc_cos)
    q5(1) = arc_cos;
    q5(2) = 2.0*pi - arc_cos;
end
%S�LECTION DE theta5
for i=1:2
    if q5(i)>pi
        q5(i)=q5(i)-2*pi;
    elseif q5(i)<-pi
        q5(i)=q5(i)+2*pi;
    end
    if abs((q5(i)-theta_act(5))) < - pi
        diff_th5(i)=abs(abs(q5(i)-theta_act(5))+2*pi);
    elseif abs((q5(i)-theta_act(5))) > pi
        diff_th5(i)=abs(abs(q5(i)-theta_act(5))-2*pi);
    else
        diff_th5(i)=abs(q5(i)-theta_act(5));
    end
end
if diff_th5(1)>diff_th5(2)
    theta_5=q5(2);
else
    theta_5=q5(1);
end
%%%%%%%%%%%%%%%%%%%%

% Q1_1=[cos(theta1_1) -sin(theta1_1)*cos(dh.alpha(1)) sin(theta1_1)*sin(dh.alpha(1));
%         sin(theta1_1) cos(theta1_1)*cos(dh.alpha(1)) -cos(theta1_1)*sin(dh.alpha(1));
%         0 sin(dh.alpha(1)) cos(dh.alpha(1))];
% Q1_2=[cos(theta1_2) -sin(theta1_2)*cos(dh.alpha(1)) sin(theta1_2)*sin(dh.alpha(1));
%     sin(theta1_2) cos(theta1_2)*cos(dh.alpha(1)) -cos(theta1_2)*sin(dh.alpha(1));
%     0 sin(dh.alpha(1)) cos(dh.alpha(1))];
% P_rep2_1=Q1_1'*Pose(1:3);
% P_rep2_2=Q1_2'*Pose(1:3);
% L_1=P_rep2_1(2)+dh.b(4)-dh.b(1);
% L_2=P_rep2_2(2)+dh.b(4)-dh.b(1);
% 
% theta5_1=pi/2+asin(L_1/dh.b(6))
% theta5_2=3*pi/2-asin(L_1/dh.b(6))
% theta5_3=pi/2+asin(L_2/dh.b(6))
% theta5_4=3*pi/2-asin(L_2/dh.b(6))


%%%%%%%%%%%%% 3e Poignet et joints RRR (q2,q3,q4 et q6) %%%%%%%%%%%%
%i j k = [1 1 1]

        c1 = cos(theta_1);
        s1 = sin(theta_1);
        c5 = cos(theta_5);
        s5 = sin(theta_5);

        %%%%%%%%%%%%%%%%%%%%% 3e Poignet (q6) %%%%%%%%%%%%%%%%%%%%%%
        if abs(s5) < ZERO_THRESH 
            %q6 = q6_des
            theta_6 = 0;
        else 
            theta_6 = atan2(sign(s5)*-(Pose(1,2)*s1 - Pose(2,2)*c1), sign(s5)*(Pose(1,1)*s1 - Pose(2,1)*c1));
            if abs(theta_6) < ZERO_THRESH 
                theta_6 = 0.0;
            end
            if theta_6 < 0.0 
                theta_6 = theta_6 + 2.0*pi;
            end
        end
        %%%%%%%%%%%%%%%% Joints RRR (q2, q3 et q4) %%%%%%%%%%%%%%%%%
        q2=[1, 0]; q3=[1, 0]; q4=[1, 0];
        c6 = cos(theta_6); s6 = sin(theta_6);
        x04x = -s5*(Pose(1,3)*c1 + Pose(2,3)*s1) - c5*(s6*(Pose(1,2)*c1 + Pose(2,2)*s1) - c6*(Pose(1,1)*c1 + Pose(2,1)*s1));
        x04y = c5*(Pose(3,1)*c6 - Pose(3,2)*s6) - Pose(3,3)*s5;
        p13x = dh.b(5)*(s6*(Pose(1,1)*c1 + Pose(2,1)*s1) + c6*(Pose(1,2)*c1 + Pose(2,2)*s1)) - dh.b(6)*(Pose(1,3)*c1 + Pose(2,3)*s1) + Pose(1,4)*c1 + Pose(2,4)*s1;
        p13y = Pose(3,4) - dh.b(1) - dh.b(6)*Pose(3,3) + dh.b(5)*(Pose(3,2)*c6 + Pose(3,1)*s6);

        c3 = (p13x*p13x + p13y*p13y - dh.a(2)*dh.a(2) - dh.a(3)*dh.a(3)) / (2.0*dh.a(2)*dh.a(3));
        if abs(abs(c3) - 1.0) < ZERO_THRESH 
            c3 = sign(c3);
        elseif abs(c3) > 1.0
            % PoseODO  NO SOLUPoseION
            % Il n'existe pas de solution dans cette branche. 
            % Passage � la prochaine it�ration de la boucle j.
            q_sols=theta_act;
        end
        arc_cos = acos(c3);
        q3(1) = arc_cos;
        q3(2) = 2.0*pi - arc_cos;
        for i=1:2
            if q3(i)>pi
                q3(i)=q3(i)-2*pi;
            elseif q3(i) < -pi
                q3(i)=q3(i)+2*pi;
            end
            if abs((q3(i)-theta_act(3))) < - pi
                diff_th3(i)=abs(abs(q3(i)-theta_act(3))+2*pi);
            elseif abs((q3(i)-theta_act(3))) > pi
                diff_th3(i)=abs(abs(q3(i)-theta_act(3))-2*pi);
            else
                diff_th3(i)=abs(q3(i)-theta_act(3));
            end
        end
        if diff_th3(1)>diff_th3(2)
            theta_3=q3(2);
            k=2;
        else
            theta_3=q3(1);
            k=1;
        end
        denom = dh.a(2)*dh.a(2) + dh.a(3)*dh.a(3) + 2*dh.a(2)*dh.a(3)*c3;
%         s3 = sin(arc_cos);
        s3=sin(theta_3);c3=cos(theta_3);
        A = dh.a(2) + dh.a(3)*c3;
        B = dh.a(3)*s3;
        if k==1
            theta_2 = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
        else
            theta_2 = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
        end
        
        
        c23_0 = cos(theta_2+theta_3);
        s23_0 = sin(theta_2+theta_3);
        theta_4 = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);

        
        
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if abs(q2(k)) < ZERO_THRESH 
            q2(k) = 0.0;
        elseif q2(k) < 0.0 
            q2(k) = q2(k)+2.0*pi;
        end

        if abs(q4(k)) < ZERO_THRESH 
            q4(k) = 0.0;
        elseif q4(k) < 0.0 
            q4(k) = q4(k)+2.0 * pi;
        end

        % Ajout de la solution (si applicable) � la liste de 
        % solution(s) possible(s)
        local_sol=theta_act;
        local_sol(1) = theta_1;
        local_sol(2) = theta_2;
        local_sol(3) = theta_3;
        local_sol(4) = theta_4;
        local_sol(5) = theta_5;
        local_sol(6) = theta_6;
        if num_sols==0 
            q_sols = local_sol;
        else 
            q_sols = [q_sols local_sol];
        end
    %on remet les angles entre -pi et pi
    for t=1:6
        if q_sols(t)>pi
            q_sols(t)=q_sols(t)-2*pi;
        elseif q_sols(t)<-pi
            q_sols(t)=q_sols(t)+2*pi;
        end
    end
end







% %______________CALCUL DE THETA 6__________________
% 
% mat_1=Q'*Q1_1;
% mat_2=Q'*Q1_2;
% if sin(theta5_1)<0.001
%     theta6_1=theta_act(6);
% else
%     theta6_1=sign(sin(theta5_1))*atan2(mat_1(2,2),-mat_1(1,2))
% end
% 
% if sin(theta5_2)<0.001
%     theta6_2=theta_act(6);
% else
%     theta6_2=sign(sin(theta5_2))*atan2(mat_1(2,2),-mat_1(1,2))
% end
% if sin(theta5_3)<0.001
%     theta6_3=theta_act(6);
% else
%     theta6_3=sign(sin(theta5_1))*atan2(mat_2(2,2),-mat_2(1,2))
% end
% if sin(theta5_4)<0.001
%     theta6_4=theta_act(6);
% else
%     theta6_4=sign(sin(theta5_2))*atan2(mat_2(2,2),-mat_2(1,2))
% end
% 
% %______________CALCUL DE THETA 3__________________
% 
% Q6_1=[cos(theta6_1) -sin(theta6_1)*cos(dh.alpha(6)) sin(theta6_1)*sin(dh.alpha(6));
%         sin(theta6_1) cos(theta6_1)*cos(dh.alpha(6)) -cos(theta6_1)*sin(dh.alpha(6));
%         0 sin(dh.alpha(6)) cos(dh.alpha(6))];
% Q6_2=[cos(theta6_2) -sin(theta6_2)*cos(dh.alpha(6)) sin(theta6_2)*sin(dh.alpha(6));
%         sin(theta6_2) cos(theta6_2)*cos(dh.alpha(6)) -cos(theta6_2)*sin(dh.alpha(6));
%         0 sin(dh.alpha(6)) cos(dh.alpha(6))];
% Q6_3=[cos(theta6_3) -sin(theta6_3)*cos(dh.alpha(6)) sin(theta6_3)*sin(dh.alpha(6));
%         sin(theta6_3) cos(theta6_3)*cos(dh.alpha(6)) -cos(theta6_3)*sin(dh.alpha(6));
%         0 sin(dh.alpha(6)) cos(dh.alpha(6))];
% Q6_4=[cos(theta6_4) -sin(theta6_4)*cos(dh.alpha(6)) sin(theta6_4)*sin(dh.alpha(6));
%         sin(theta6_4) cos(theta6_4)*cos(dh.alpha(6)) -cos(theta6_4)*sin(dh.alpha(6));
%         0 sin(dh.alpha(6)) cos(dh.alpha(6))];
% Q5_1=[cos(theta5_1) -sin(theta5_1)*cos(dh.alpha(5)) sin(theta5_1)*sin(dh.alpha(5));
%         sin(theta5_1) cos(theta5_1)*cos(dh.alpha(5)) -cos(theta5_1)*sin(dh.alpha(5));
%         0 sin(dh.alpha(5)) cos(dh.alpha(5))];
% Q5_2=[cos(theta5_2) -sin(theta5_2)*cos(dh.alpha(5)) sin(theta5_2)*sin(dh.alpha(5));
%         sin(theta5_2) cos(theta5_2)*cos(dh.alpha(5)) -cos(theta5_2)*sin(dh.alpha(5));
%         0 sin(dh.alpha(5)) cos(dh.alpha(5))];
% Q5_3=[cos(theta5_3) -sin(theta5_3)*cos(dh.alpha(5)) sin(theta5_3)*sin(dh.alpha(5));
%         sin(theta5_3) cos(theta5_3)*cos(dh.alpha(5)) -cos(theta5_3)*sin(dh.alpha(5));
%         0 sin(dh.alpha(5)) cos(dh.alpha(5))];
% Q5_4=[cos(theta5_4) -sin(theta5_4)*cos(dh.alpha(5)) sin(theta5_4)*sin(dh.alpha(5));
%         sin(theta5_4) cos(theta5_4)*cos(dh.alpha(5)) -cos(theta5_4)*sin(dh.alpha(5));
%         0 sin(dh.alpha(5)) cos(dh.alpha(5))];
% 
%     
% a1_1=[dh.a(1)*cos(theta1_1);dh.a(1)*sin(theta1_1);dh.b(1)];
% a1_2=[dh.a(1)*cos(theta1_2);dh.a(1)*sin(theta1_2);dh.b(1)];
% a5_1=[dh.a(5)*cos(theta5_1);dh.a(6)*sin(theta5_1);dh.b(5)];
% a5_2=[dh.a(5)*cos(theta5_2);dh.a(6)*sin(theta5_2);dh.b(5)];
% a5_3=[dh.a(5)*cos(theta5_3);dh.a(6)*sin(theta5_3);dh.b(5)];
% a5_4=[dh.a(5)*cos(theta5_4);dh.a(6)*sin(theta5_4);dh.b(5)];
% a6_1=[dh.a(6)*cos(theta6_1);dh.a(6)*sin(theta6_1);dh.b(6)];
% a6_2=[dh.a(6)*cos(theta6_2);dh.a(6)*sin(theta6_2);dh.b(6)];
% a6_3=[dh.a(6)*cos(theta6_3);dh.a(6)*sin(theta6_3);dh.b(6)];
% a6_4=[dh.a(6)*cos(theta6_4);dh.a(6)*sin(theta6_4);dh.b(6)];
% 
% 
% P4_rep2_1=Q1_1'*(Pose(1:3)-Q6_1'*Q*a6_1-Q5_1'*Q6_1'*Q*a5_1-a1_1);
% P4_rep2_2=Q1_1'*(Pose(1:3)-Q6_2'*Q*a6_2-Q5_2'*Q6_2'*Q*a5_2-a1_1);
% P4_rep2_3=Q1_2'*(Pose(1:3)-Q6_3'*Q*a6_3-Q5_3'*Q6_3'*Q*a5_3-a1_2);
% P4_rep2_4=Q1_2'*(Pose(1:3)-Q6_4'*Q*a6_4-Q5_4'*Q6_4'*Q*a5_4-a1_2);
% 
% r_1=sqrt(P4_rep2_1(1)^2+P4_rep2_1(2)^2);
% r_2=sqrt(P4_rep2_2(1)^2+P4_rep2_2(2)^2);
% r_3=sqrt(P4_rep2_3(1)^2+P4_rep2_3(2)^2);
% r_4=sqrt(P4_rep2_4(1)^2+P4_rep2_4(2)^2);
% 
% theta3_1=pi-acos((r_1^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_2=pi+acos((r_1^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_3=pi-acos((r_2^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_4=pi+acos((r_2^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_5=pi-acos((r_3^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_6=pi+acos((r_3^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_7=pi-acos((r_4^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% theta3_8=pi+acos((r_4^2-dh.a(2)^2-dh.a(3)^2)/(-2*dh.a(2)*dh.a(3)));
% 
% %______________CALCUL DE THETA 2__________________
% 
% epsilon_1=atan2(P4_rep2_1(3),P4_rep2_1(1));
% epsilon_2=atan2(P4_rep2_2(3),P4_rep2_2(1));
% epsilon_3=atan2(P4_rep2_3(3),P4_rep2_3(1));
% epsilon_4=atan2(P4_rep2_4(3),P4_rep2_4(1));
% c_1=asin(dh.a(3)*sin(pi-theta3_1)/r_1);
% c_2=asin(dh.a(3)*sin(pi-theta3_2)/r_1);
% c_3=asin(dh.a(3)*sin(pi-theta3_3)/r_2);
% c_4=asin(dh.a(3)*sin(pi-theta3_4)/r_2);
% c_5=asin(dh.a(3)*sin(pi-theta3_5)/r_3);
% c_6=asin(dh.a(3)*sin(pi-theta3_6)/r_3);
% c_7=asin(dh.a(3)*sin(pi-theta3_7)/r_4);
% c_8=asin(dh.a(3)*sin(pi-theta3_8)/r_4);
% 
% theta2_1=epsilon_1-sign(sin(theta3_1))*c_1+pi;
% theta2_2=epsilon_1-sign(sin(theta3_2))*c_2+pi;
% theta2_3=epsilon_2-sign(sin(theta3_3))*c_3+pi;
% theta2_4=epsilon_2-sign(sin(theta3_4))*c_4+pi;
% theta2_5=epsilon_3-sign(sin(theta3_5))*c_5+pi;
% theta2_6=epsilon_3-sign(sin(theta3_6))*c_6+pi;
% theta2_7=epsilon_4-sign(sin(theta3_7))*c_7+pi;
% theta2_8=epsilon_4-sign(sin(theta3_8))*c_8+pi;
% 
% 
% %______________CALCUL DE THETA 4__________________
% 
% Q2_1=[cos(theta2_1) -sin(theta2_1)*cos(dh.alpha(2)) sin(theta2_1)*sin(dh.alpha(2));
%         sin(theta2_1) cos(theta2_1)*cos(dh.alpha(2)) -cos(theta2_1)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_2=[cos(theta2_2) -sin(theta2_2)*cos(dh.alpha(2)) sin(theta2_2)*sin(dh.alpha(2));
%         sin(theta2_2) cos(theta2_2)*cos(dh.alpha(2)) -cos(theta2_2)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_3=[cos(theta2_3) -sin(theta2_3)*cos(dh.alpha(2)) sin(theta2_3)*sin(dh.alpha(2));
%         sin(theta2_3) cos(theta2_3)*cos(dh.alpha(2)) -cos(theta2_3)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_4=[cos(theta2_4) -sin(theta2_4)*cos(dh.alpha(2)) sin(theta2_4)*sin(dh.alpha(2));
%         sin(theta2_4) cos(theta2_4)*cos(dh.alpha(2)) -cos(theta2_4)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_5=[cos(theta2_5) -sin(theta2_5)*cos(dh.alpha(2)) sin(theta2_5)*sin(dh.alpha(2));
%         sin(theta2_5) cos(theta2_5)*cos(dh.alpha(2)) -cos(theta2_5)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_6=[cos(theta2_6) -sin(theta2_6)*cos(dh.alpha(2)) sin(theta2_6)*sin(dh.alpha(2));
%         sin(theta2_6) cos(theta2_6)*cos(dh.alpha(2)) -cos(theta2_6)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_7=[cos(theta2_7) -sin(theta2_7)*cos(dh.alpha(2)) sin(theta2_7)*sin(dh.alpha(2));
%         sin(theta2_7) cos(theta2_7)*cos(dh.alpha(2)) -cos(theta2_7)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
% Q2_8=[cos(theta2_8) -sin(theta2_8)*cos(dh.alpha(2)) sin(theta2_8)*sin(dh.alpha(2));
%         sin(theta2_8) cos(theta2_8)*cos(dh.alpha(2)) -cos(theta2_8)*sin(dh.alpha(2));
%         0 sin(dh.alpha(2)) cos(dh.alpha(2))];
%     
%     
% Q3_1=[cos(theta3_1) -sin(theta3_1)*cos(dh.alpha(3)) sin(theta3_1)*sin(dh.alpha(3));
%         sin(theta3_1) cos(theta3_1)*cos(dh.alpha(3)) -cos(theta3_1)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_2=[cos(theta3_2) -sin(theta3_2)*cos(dh.alpha(3)) sin(theta3_2)*sin(dh.alpha(3));
%         sin(theta3_2) cos(theta3_2)*cos(dh.alpha(3)) -cos(theta3_2)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_3=[cos(theta3_3) -sin(theta3_3)*cos(dh.alpha(3)) sin(theta3_3)*sin(dh.alpha(3));
%         sin(theta3_3) cos(theta3_3)*cos(dh.alpha(3)) -cos(theta3_3)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_4=[cos(theta3_4) -sin(theta3_4)*cos(dh.alpha(3)) sin(theta3_4)*sin(dh.alpha(3));
%         sin(theta3_4) cos(theta3_4)*cos(dh.alpha(3)) -cos(theta3_4)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_5=[cos(theta3_5) -sin(theta3_5)*cos(dh.alpha(3)) sin(theta3_5)*sin(dh.alpha(3));
%         sin(theta3_5) cos(theta3_5)*cos(dh.alpha(3)) -cos(theta3_5)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_6=[cos(theta3_6) -sin(theta3_6)*cos(dh.alpha(3)) sin(theta3_6)*sin(dh.alpha(3));
%         sin(theta3_6) cos(theta3_6)*cos(dh.alpha(3)) -cos(theta3_6)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_7=[cos(theta3_7) -sin(theta3_7)*cos(dh.alpha(3)) sin(theta3_7)*sin(dh.alpha(3));
%         sin(theta3_7) cos(theta3_7)*cos(dh.alpha(3)) -cos(theta3_7)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% Q3_8=[cos(theta3_8) -sin(theta3_8)*cos(dh.alpha(3)) sin(theta3_8)*sin(dh.alpha(3));
%         sin(theta3_8) cos(theta3_8)*cos(dh.alpha(3)) -cos(theta3_8)*sin(dh.alpha(3));
%         0 sin(dh.alpha(3)) cos(dh.alpha(3))];
% 
% Q4_1=Q6_1'*Q5_1'*Q*Q1_1'*Q2_1'*Q3_1';
% Q4_2=Q6_1'*Q5_1'*Q*Q1_1'*Q2_2'*Q3_2';
% Q4_3=Q6_2'*Q5_2'*Q*Q1_1'*Q2_3'*Q3_3';
% Q4_4=Q6_2'*Q5_2'*Q*Q1_1'*Q2_4'*Q3_4';
% Q4_5=Q6_3'*Q5_3'*Q*Q1_2'*Q2_5'*Q3_5';
% Q4_6=Q6_3'*Q5_3'*Q*Q1_2'*Q2_6'*Q3_6';
% Q4_7=Q6_4'*Q5_4'*Q*Q1_2'*Q2_7'*Q3_7';
% Q4_8=Q6_4'*Q5_4'*Q*Q1_2'*Q2_8'*Q3_8';
% 
% theta4_1=atan2(Q4_1(2,1),Q4_1(1,1));
% theta4_2=atan2(Q4_2(2,1),Q4_2(1,1));
% theta4_3=atan2(Q4_3(2,1),Q4_3(1,1));
% theta4_4=atan2(Q4_4(2,1),Q4_4(1,1));
% theta4_5=atan2(Q4_5(2,1),Q4_5(1,1));
% theta4_6=atan2(Q4_6(2,1),Q4_6(1,1));
% theta4_7=atan2(Q4_7(2,1),Q4_7(1,1));
% theta4_8=atan2(Q4_8(2,1),Q4_8(1,1));
% 
% 
% 
% theta=[theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
%         theta1_1 theta2_2 theta3_2 theta4_2 theta5_1 theta6_1;
%         theta1_1 theta2_3 theta3_3 theta4_3 theta5_2 theta6_2;
%         theta1_1 theta2_4 theta3_4 theta4_4 theta5_2 theta6_2;
%         theta1_2 theta2_5 theta3_5 theta4_5 theta5_3 theta6_3;
%         theta1_2 theta2_6 theta3_6 theta4_6 theta5_3 theta6_3;
%         theta1_2 theta2_7 theta3_7 theta4_7 theta5_4 theta6_4;
%         theta1_2 theta2_8 theta3_8 theta4_8 theta5_4 theta6_4];






















