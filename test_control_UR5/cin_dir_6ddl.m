function Pose = cin_dir_6ddl( q, dh)
    %cin_dir ressud le probleme geometrique directe du robot RRRRRR
    %Output: 
    %   Pose: liste des poses (3x1) de chacun des reperes (P1 P2 P3 P4 P5 P6 P7)'
    %   euler: lest des angles d'euler dur repere 7 p/r au repere 1 (3x1)
    %   Q: cell array (Q Q1 Q2 Q3 Q4 Q5 Q6)
    
%         Fonction resolvant le probleme geometrique direct (PGD) de
%         l'UR5 a partir de ses positions articulaires (q). Retourne la
%         matrice T a l'outil et une matrice "p" contenant les positions
%         cartesiennes de toutes les articulations p = ((x0 x1 x2 x3 x4 x5;
%                                                        y0 y1 y2 y3 y4 y5;
%                                                        z0 z1 z2 z3 z4 z5))
        ZERO_THRESH=10^-8;
        p = zeros(3,6);
        Q = [1 0 0;0 1 0;0 0 1];
        c=cos(q);
        s=sin(q);

        for i=1:6
            A = [dh.a(i)*c(i),dh.a(i)*s(i),dh.b(i)]';

            % Position de l'articulation
            if i ~= 1
                p(:,i) = p(:,i-1) + Q*A;
            else
                p(:,i) = Q*A;
            end
            % Matrices de rotation locale Q_i
            if i==1 || i==4 % alpha = pi/2
                Q_i =[c(i), 0 ,  s(i);
                     s(i), 0 , -c(i);
                     0   , 1 ,  0];

            elseif (i==2 || i==3) || i==6 % alpha = 0
                Q_i = [c(i),-s(i), 0;
                       s(i), c(i), 0;
                       0   , 0   , 1];

            else % alpha = -pi/2
                Q_i = [c(i), 0 , -s(i);
                       s(i), 0 ,  c(i);
                       0   , -1 ,  0];
            end
            % Matrice de rotation cumulative (Q -> Q_i)
            Q = Q*Q_i;
        end

        % Construction de la matrice T
        T=zeros(4,4); T(1:3,1:3)=Q; T(1:3,4)=p(:,end); T(4,4)=1;
        T(abs(T)<ZERO_THRESH)=0; % Retirer les tres petites valeurs
        Pose=T;
    
%     theta=theta_act+dh.theta;
%     Q=(1 0 0;0 1 0;0 0 1);
%     Q_temp= Q;
%     P=(0;0;0);
%     Pose=();
%     for i=1:length(dh.a)
%         a=(dh.a(i)*cos(theta(i));
%             dh.a(i)*sin(theta(i));
%             dh.b(i));
%         Q=(cos(theta(i)) -sin(theta(i))*cos(dh.alpha(i)) sin(theta(i))*sin(dh.alpha(i));
%                     sin(theta(i)) cos(theta(i))*cos(dh.alpha(i)) -cos(theta(i))*sin(dh.alpha(i));
%                     0 sin(dh.alpha(i)) cos(dh.alpha(i)));
%         P=P+Q_temp*a;
%         Q_temp=Q_temp*Q;
%         
% %         P=(a(2)*cos(theta(1))*cos(theta(2))+a(3)*cos(theta(1))*cos(theta(2)+theta(3))+b(4)*sin(theta(1))+b(5)*cos(theta(1))*cos(theta(2)+theta(3)+theta(4))+b(6)*(-cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))+sin(theta(1))*cos(theta(5)));
% %             a(2)*sin(theta(1))*cos(theta(2))+a(3)*sin(theta(1))*cos(theta(2)+theta(3))-b(4)*cos(theta(1))+b(5)*sin(theta(1))*sin(theta(2)+theta(3)+theta(4))+b(6)*(-sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))-cos(theta(1))*cos(theta(5)));
% %            b(1)+a(2)*sin(theta(2))+a(3)*sin(theta(2)+theta(3))-b(5)*cos(theta(2)+theta(3)+theta(4))-b(6)*sin(theta(2)+theta(3)+theta(4))*sin(theta(5)));
% %        r11=cos(theta(6))*(cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))+sin(theta(1))*sin(theta(5)))+sin(theta(6))*(-cos(theta(1))*sin(theta(2)+theta(3)+theta(4)));
% %         r12=-sin(theta(6))*(cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))+sin(theta(1))*sin(theta(5)))+cos(theta(6))*(-cos(theta(1))*sin(theta(2)+theta(3)+theta(4)));
% %         r13=-cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))+sin(theta(1))*cos(theta(5));
% %        r21=cos(theta(6))*(sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))-cos(theta(1))*sin(theta(5)))+sin(theta(6))*(-sin(theta(1))*sin(theta(2)+theta(3)+theta(4)));
% %         r22=-sin(theta(6))*(sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))-cos(theta(1))*sin(theta(5)))+cos(theta(6))*(-sin(theta(1))*sin(theta(2)+theta(3)+theta(4)));
% %         r23=-sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))-cos(theta(1))*cos(theta(5));
% %        r31=cos(theta(6))*(sin(theta(2)+theta(3)+theta(4))*cos(theta(5)))+sin(theta(6))*cos(theta(2)+theta(3)+theta(4));
% %        r32=-sin(theta(6))*(sin(theta(2)+theta(3)+theta(4))*cos(theta(5)))+cos(theta(6))*cos(theta(2)+theta(3)+theta(4));
% %        r33=-sin(theta(2)+theta(3)+theta(4))*sin(theta(5));
% 
%     end
% %     euler2=atan2(sqrt(1-Q_temp(3,3)^2),Q_temp(3,3));
% %     euler1=atan2(Q_temp(2,3)/sin(euler2),Q_temp(1,3)/sin(euler2));
% %     euler3=atan2(Q_temp(3,2)/sin(euler2),-Q_temp(3,1)/sin(euler2));
% %     euler=(euler1;
% %             euler2;
% %             euler3);
%     Pose=(Q_temp P;0 0 0 1);
% end

