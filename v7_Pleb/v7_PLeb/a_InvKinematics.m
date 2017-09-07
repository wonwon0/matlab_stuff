function [thetaout,error] = a_InvKinematics(theta,posDes)

global Ang_Velocity_Max;


% posDes = [-0.0098;0.4054;0.2802];
% posDes = [-0.0098;-0.5733;0.2755];
% theta = [0;90;0]*pi/180;


    error = 0;
    error_1 = 0;
    error_2 = 0;
    flagErreur = 0;

    [DHa,DHb,DHal,DHth] = a_paramDH();
    [LimitAng] = a_LimitAng();

    D1=DHb(1);
    D2=DHa(2);
    D3=DHa(3);
    e2 = DHb(3);
    

    thetainRobot = theta;
    
    Erreur=[0;0;0;0];
    
    pxdes = posDes(1);
    pydes = posDes(2);
    pzdes = posDes(3);
    L=[D1 D2 D3 -e2];
    x_d=pxdes;
    y_d=pydes;
    z_d=pzdes;
    %----------------------CALCUL THETA1-------------------
    %verfi simgulartie
    if ((x_d^2+y_d^2-L(4))<0.0001)
        theta_11=0;
        theta_12=0;
    else
        theta_11 = 2*atan2(-x_d+sqrt(x_d^2+y_d^2-L(4)^2),y_d+L(4));
        theta_12 = -2*atan2(x_d+sqrt(x_d^2+y_d^2-L(4)^2),y_d+L(4));

    end

    %----------------------CALCUL THETA3-------------------
    %verif singularite avec theta_11
    t1=x_d*cos(theta_11)+y_d*sin(theta_11);
    t2=z_d-L(1);
    t3=(t1^2+t2^2-L(2)^2-L(3)^2)/(2*L(2)*L(3));
    if t3^2>0.9999
        theta_31=0;
        theta_32=0;
    else
        theta_31=atan2(sqrt(1-t3^2),t3);
        theta_32=atan2(-sqrt(1-t3^2),t3);
    end
    %verif singularite avec theta_12
    t1=x_d*cos(theta_12)+y_d*sin(theta_12);
    t2=z_d-L(1);
    t3=(t1^2+t2^2-L(2)^2-L(3)^2)/(2*L(2)*L(3));
    if t3^2>0.9999
        theta_33=0;
        theta_34=0;
    else
        theta_33=atan2(sqrt(1-t3^2),t3);
        theta_34=atan2(-sqrt(1-t3^2),t3);
    end
    %----------------------CALCUL THETA2-------------------
    %verif singularite avec theta_11 et theta_31
    t1=x_d*cos(theta_11)+y_d*sin(theta_11);
    t2=z_d-L(1);
    t3=sin(theta_31)*L(3);
    t4=L(2)+L(3)*cos(theta_31);
    %calcul theta_21 avec theta_11 et theta_31
    if sqrt((L(2)+L(3)*cos(theta_31))^2+(L(3)*sin(theta_31))^2)<0.00001
        theta_21=0;
    else
        theta_21=atan2(t2,t1)-atan2(t3,t4);
    end
    %verif singularite avec theta_11 et theta_32
    t1=x_d*cos(theta_11)+y_d*sin(theta_11);
    t2=z_d-L(1);
    t3=sin(theta_32)*L(3);
    t4=L(2)+L(3)*cos(theta_32);
    %calcul theta_22 avec theta_11 et theta_32
    if sqrt((L(2)+L(3)*cos(theta_32))^2+(L(3)*sin(theta_32))^2)<0.00001
        theta_22=0;
    else
        theta_22=atan2(t2,t1)-atan2(t3,t4);
    end
    %verif singularite avec theta_12 et theta_33
    t1=x_d*cos(theta_12)+y_d*sin(theta_12);
    t2=z_d-L(1);
    t3=sin(theta_33)*L(3);
    t4=L(2)+L(3)*cos(theta_33);
    %calcul theta_23 avec theta_12 et theta_33
    if sqrt((L(2)+L(3)*cos(theta_33))^2+(L(3)*sin(theta_33))^2)<0.00001
        theta_23=0;
    else
        theta_23=atan2(t2,t1)-atan2(t3,t4);
    end
    %verif singularite avec theta_12 et theta_34
    t1=x_d*cos(theta_12)+y_d*sin(theta_12);
    t2=z_d-L(1);
    t3=sin(theta_34)*L(3);
    t4=L(2)+L(3)*cos(theta_34);
    %calcul theta_24 avec theta_12 et theta_34
    if sqrt((L(2)+L(3)*cos(theta_34))^2+(L(3)*sin(theta_34))^2)<0.00001
        theta_24=0;
    else
        theta_24=atan2(t2,t1)-atan2(t3,t4);
    end
    %on enlève les angles du a la config du robot quand on a calculer les param
    %dh
    theta1=[theta_11 theta_11 theta_12 theta_12]';
    theta2=[theta_21 theta_22 theta_23 theta_24]';
    theta3=[theta_31 theta_32 theta_33 theta_34]';
    Solution=[theta1 theta2 theta3]';
%{
    % Radius of the desired position
    r1 = sqrt(pxdes^2+pydes^2);
    
		% If the desired position is near the singularity
		if ( abs(r1) < abs(e2*1.1) )
			for er=1:4
				theta1_t11 = 0;
				theta1_t12 = 0;
				Erreur(er) = 1;
            end
		else
		% else we compute theta1
		    theta1_t11 = 2*atan2(-pxdes+sqrt(pxdes^2+pydes^2-e2^2),pydes-e2);
		    theta1_t12 = -2*atan2(pxdes+sqrt(pxdes^2+pydes^2-e2^2),pydes-e2);
        end
        

		% Intermediate computation
		F_t11 = pxdes*cos(theta1_t11)+pydes*sin(theta1_t11);
		F_t12 = pxdes*cos(theta1_t12)+pydes*sin(theta1_t12);

		N_t11 = F_t11*F_t11 + (pzdes-D1)*(pzdes-D1);
		N_t12 = F_t12*F_t12 + (pzdes-D1)*(pzdes-D1);

		w_t11 = -(D3*D3+D2*D2-N_t11) / (2*D3*D2);
		w_t12 = -(D3*D3+D2*D2-N_t12) / (2*D3*D2);
        
        
		% singularity management
		if ( abs(w_t11) > 0.9999 )
		% If singularity flag error for this solution
		    theta3_t11_t31 = 0;
		    theta3_t11_t32 = 0;
		    Erreur(1) = 1;
		    Erreur(2) = 1;
		else
		% Compute theta3 solution 1
		    theta3_t11_t31 = atan2(sqrt(1-w_t11*w_t11), w_t11);
		    theta3_t11_t32 = atan2(-sqrt(1-w_t11*w_t11), w_t11);
        end

		% singularity management
		if ( abs(w_t12) > 0.9999 )
		% If singularity flag error for this solution
		    theta3_t12_t31 = 0;
		    theta3_t12_t32 = 0;
		    Erreur(3) = 1;
		    Erreur(4) = 1;
		else
		% Compute theta3 solution 2
		    theta3_t12_t31 = atan2(sqrt(1-w_t12*w_t12), w_t12);
		    theta3_t12_t32 = atan2(-sqrt(1-w_t12*w_t12), w_t12);
        end
        
		% Intermediate computation
		H_t11_t31 = D2+D3*cos(theta3_t11_t31);
		H_t11_t32 = D2+D3*cos(theta3_t11_t32);
		H_t12_t31 = D2+D3*cos(theta3_t12_t31);
		H_t12_t32 = D2+D3*cos(theta3_t12_t32);

		G_t11_t31 = D3*sin(theta3_t11_t31);
		G_t11_t32 = D3*sin(theta3_t11_t32);
		G_t12_t31 = D3*sin(theta3_t12_t31);
		G_t12_t32 = D3*sin(theta3_t12_t32);

		L_t11_t31 = pxdes*cos(theta1_t11) + pydes*sin(theta1_t11);
		L_t11_t32 = pxdes*cos(theta1_t11) + pydes*sin(theta1_t11);
		L_t12_t31 = pxdes*cos(theta1_t12) + pydes*sin(theta1_t12);
		L_t12_t32 = pxdes*cos(theta1_t12) + pydes*sin(theta1_t12);

		M_t11_t31 = pzdes-D1;
		M_t11_t32 = pzdes-D1;
		M_t12_t31 = pzdes-D1;
		M_t12_t32 = pzdes-D1;
        
        
		% Compute theta 2
		if ( sqrt(abs(H_t11_t31*H_t11_t31+G_t11_t31*G_t11_t31)) < 0.00001 )
		% If singularity raise flag
		    theta2_t11_t31 = 0;
		    Erreur(1) = 1;
		else
		% Compute theta 2 solution 1
		    theta2_t11_t31 = atan2(M_t11_t31,L_t11_t31)-atan2(G_t11_t31,H_t11_t31);
        end
        
		if ( sqrt(abs(H_t11_t32*H_t11_t32+G_t11_t32*G_t11_t32)) < 0.00001 )
		% If singularity raise flag
		    theta2_t11_t32 = 0;
		    Erreur(2) = 1;
		else
		% Compute theta 2 solution 2
		    theta2_t11_t32 = atan2(M_t11_t32,L_t11_t32)-atan2(G_t11_t32,H_t11_t32);
        end
        
		if ( sqrt(abs(H_t12_t31*H_t12_t31+G_t12_t31*G_t12_t31)) < 0.00001 )
		% If singularity raise flag
		    theta2_t12_t31 = 0;
		    Erreur(3) = 1;
		else
		% Compute theta 2 solution 3
		    theta2_t12_t31 = atan2(M_t12_t31,L_t12_t31)-atan2(G_t12_t31,H_t12_t31);
        end

		if ( sqrt(abs(H_t12_t32*H_t12_t32+G_t12_t32*G_t12_t32)) < 0.00001 )
		% If singularity raise flag
		    theta2_t12_t32 = 0;
		    Erreur(4) = 1;
		else
		% Compute theta 2 solution 4
		    theta2_t12_t32 = atan2(M_t12_t32,L_t12_t32)-atan2(G_t12_t32,H_t12_t32);
        end
        
		% Get all the possible solutions together in a table to analyze
		Solution(1,0+1) = theta1_t11;
		Solution(1,1+1) = theta1_t11;
		Solution(1,2+1) = theta1_t12;
		Solution(1,3+1) = theta1_t12;

		Solution(2,0+1) = theta2_t11_t31;
		Solution(2,1+1) = theta2_t11_t32;
		Solution(2,2+1) = theta2_t12_t31;
		Solution(2,3+1) = theta2_t12_t32;

		Solution(3,0+1) = theta3_t11_t31;
		Solution(3,1+1) = theta3_t11_t32;
		Solution(3,2+1) = theta3_t12_t31;
		Solution(3,3+1) = theta3_t12_t32;
%}
        
        
		% Transfer the solution into robot angles
		for h=1:4
			for  g=1:3
				tempAngles(g,1) = Solution(g,h);
            end

			tempAnglesRobot = tempAngles - DHth;

			for  g=1:3
				SolutionRobot(g,h) = tempAnglesRobot(g);
            end
        end

        % Choix solution
        NbSolCoudeHaut = 0;
        IndexSolution1 = 1;
        IndexSolution2 = 1;
        IndexSolutionF = 1;
        Solution1 = [0;0;0;0;0;0];
        Solution2 = [0;0;0;0;0;0];

        % Reset the angles between 0 and 360
        for h=1:4
            if (SolutionRobot(1,h) < -pi)
                SolutionRobot(1,h) = SolutionRobot(1,h)+2*pi;
            end
            if (SolutionRobot(2,h) < -pi)
                SolutionRobot(2,h) = SolutionRobot(2,h)+2*pi;
            end
            if (SolutionRobot(1,h) > pi)
                SolutionRobot(1,h) = SolutionRobot(1,h)+2*pi;
            end
            if (SolutionRobot(2,h) > pi)
                SolutionRobot(2,h) = SolutionRobot(2,h)+2*pi;
            end
        end
        

        % Find the solutions with elbow up
        for h=1:4
            if ( (SolutionRobot(3,h) > -pi && SolutionRobot(3,h) < 0 ) )
                if (NbSolCoudeHaut == 0)
                    for g=1:3
                        Solution1(g) = SolutionRobot(g,h);
                    end
                    IndexSolution1 = h;
                else
                    for g=1:3
                        Solution2(g) = SolutionRobot(g,h);
                    end
                    IndexSolution2 = h;
                end
                NbSolCoudeHaut = NbSolCoudeHaut+1;
            end
        end
        
        % Number of elbow up solutions should be 2
        if (NbSolCoudeHaut ~= 2)
            flagErreur = 1;
        end
        
        % Find the error for joint 1 between different solutions
        Theta1_Err1 = Solution1(1) - thetainRobot(1);
        Theta1_Err2 = Solution2(1) - thetainRobot(1);

        % Find the solution nearest to the actual position and refactor between 0 and 360
        if (Theta1_Err1 > 0)
            Theta1_Err1_360 = Theta1_Err1 - floor(abs(Theta1_Err1)/(2*pi))*(2*pi);
        else
            Theta1_Err1_360 = Theta1_Err1 + floor(abs(Theta1_Err1)/(2*pi))*(2*pi);
        end
        if (Theta1_Err1_360 < -pi)
            Theta1_Err1_360 = Theta1_Err1_360 + (2*pi);
        elseif (Theta1_Err1_360 > pi)
            Theta1_Err1_360 = Theta1_Err1_360 - (2*pi);
        end

        if (Theta1_Err2 > 0)
            Theta1_Err2_360 = Theta1_Err2 - floor(abs(Theta1_Err2)/(2*pi))*(2*pi);
        else
            Theta1_Err2_360 = Theta1_Err2 + floor(abs(Theta1_Err2)/(2*pi))*(2*pi);
        end
        if (Theta1_Err2_360 < -pi)
            Theta1_Err2_360 = Theta1_Err2_360 + (2*pi);
        elseif (Theta1_Err2_360 > pi)
            Theta1_Err2_360 = Theta1_Err2_360 - (2*pi);
        end
        
        % Find the closest solution to the actual position
        if (abs(Theta1_Err1_360) < abs(Theta1_Err2_360))
            SolutionF = Solution1;
            IndexSolutionF = IndexSolution1;
        else
            SolutionF = Solution2;
            IndexSolutionF = IndexSolution2;
        end
        
        % If error
        if (Erreur(IndexSolutionF) ~= 0)
            flagErreur = 1;
        end
        
        % Rotation infinie
        % Infinite rotation: Get everything back in the absolute range (not 0-360)
        DiffTheta1 = SolutionF(1) - thetainRobot(1);
        DiffTheta1_360 =  DiffTheta1;
        if (DiffTheta1 > 0)
            DiffTheta1_360 = DiffTheta1 - floor(abs(DiffTheta1)/(2*pi))*(2*pi);
        else
            DiffTheta1_360 = DiffTheta1 + floor(abs(DiffTheta1)/(2*pi))*(2*pi);
        end

        if (DiffTheta1_360 < -pi)
            DiffTheta1_360 = DiffTheta1_360 + (2*pi);
        elseif (DiffTheta1_360 > pi)
            DiffTheta1_360 = DiffTheta1_360 - (2*pi);
        end
            
            
        theta_IK(1,1) = thetainRobot(1) + DiffTheta1_360;
        theta_IK(2,1) = SolutionF(2);
        theta_IK(3,1) = SolutionF(3);
            
        
        if theta_IK(2)<-pi
            theta_IK(2)=theta_IK(2)+2*pi;
        end
        if theta_IK(2)>pi
            theta_IK(2)=theta_IK(2)-2*pi;
        end
        
        if theta_IK(3)<-pi
            theta_IK(3)=theta_IK(3)+2*pi;
        end
        if theta_IK(3)>pi
            theta_IK(3)=theta_IK(3)-2*pi;
        end
        
		% If DeltaTheta is too big, raise a flag
        DeltaTheta = theta_IK - thetainRobot;
		for h=1:3
			if (DeltaTheta(h) > 10)
				flagErreur = 1;
            end
        end
        
        thetaout1 = theta_IK(1);
        thetaout2 = theta_IK(2);
        thetaout3 = theta_IK(3);
        
        % Maximum angular velocity
        maxthetainc=Ang_Velocity_Max;
        if norm(DeltaTheta)>maxthetainc
            thetaout1 = thetainRobot(1) + DeltaTheta(1)*maxthetainc/norm(DeltaTheta);
            thetaout2 = thetainRobot(2) + DeltaTheta(2)*maxthetainc/norm(DeltaTheta);
            thetaout3 = thetainRobot(3) + DeltaTheta(3)*maxthetainc/norm(DeltaTheta);
        end
        
        
        [T,T1,T2,T3] = a_CinDirecte([thetaout1;thetaout2;thetaout3]);
        
        
        % JOINT POSITION LIMITATION
            if (thetaout2 < LimitAng(2,1) || thetaout2 > LimitAng(2,2))
                error=1;
            end
            if (thetaout3 < LimitAng(3,1) || thetaout3 > LimitAng(3,2))
                error=1;
            end
        
        % BASE PROTECTION
            global Base_H1;
            global Base_H2;          
            global Base_H3;   
            global Base_H4;    
            global Base_Cyl1;
            global Hard_cylindreInt;

            [Limitation,inside] = a_ProtectionBase3D(T(1:3,4),[],Base_H1,Base_H2,Base_H3,Base_H4,Base_Cyl1,Hard_cylindreInt);

            if (inside==1)
                error=1;
            end
        
        % EXTERNAL SPHERE PROTECTION
            if sqrt(T(1,4)^2+T(2,4)^2+(T(3,4)-DHb(1))^2) >= (DHa(2)+DHa(3))
                error=1;
            end
        
        
        % PROTECTION ZONE
            [Prisms, Cylinders, Spheres, Cones] = a_ProtectionZoneDef_3D();
            [Limitation, inside, minv] = a_ProtectionZonePrisms3D(T(1:3,4), Prisms, 0, []);
            if inside == 1
                error=1;
            end
            
            Ttemp=T1*T2;
            [Limitation, inside_elbow, minv] = a_ProtectionZonePrisms3D_elbow([Ttemp(1,4);Ttemp(2,4);Ttemp(3,4)], theta, Prisms, 0, []);
            if inside_elbow == 1
                error=1;
            end
            
            
        % CHECK IF A POINT ON THE LINK IS IN THE PROTECTION ZONE    
            global active_links
            insidememb1 = 0;
            insidememb2 = 0;
            if active_links == 1

                for i=1:20
                    point1 = [0;0;DHb(1)];
                    Ttemp=T1*T2;
                    point2 = [Ttemp(1,4);Ttemp(2,4);Ttemp(3,4)];

                    point = point1 + (point2-point1)*i/20;

                    [Limitation, insidememb] = a_ProtectionZonePrisms2D(point, Prisms, 0, []);
                    if insidememb == 1
                        insidememb1 = 1;
                    end
                end
                if insidememb1 == 1
                    error=1;
                end

                for i=1:20
                    point1 = [Ttemp(1,4);Ttemp(2,4);Ttemp(3,4)];
                    point2 = [T(1,4);T(2,4);T(3,4)];

                    point = point1 + (point2-point1)*i/20;

                    [Limitation, insidememb] = a_ProtectionZonePrisms2D(point, Prisms, 0, []);
                    if insidememb == 1
                        insidememb2 = 1;
                    end
                end

                if insidememb2 == 1
                    error=1;
                end
            end
            
            
            

%             if inside == 1 || insideelbow==1 || insidememb1==1 || insidememb2 == 1
%                 error=1;
%             end      
            
        % VERIFIER ERREURS
        
        
        
        
        
    if error==1 || flagErreur==1
        thetaout1=thetainRobot(1);
        thetaout2=thetainRobot(2);
        thetaout3=thetainRobot(3);
    end
    
%     thetaOutf(1)=thetaout1 - DHth(1);
%     thetaOutf(2)=thetaout2 - DHth(2);
%     thetaOutf(3)=thetaout3 - DHth(3);
    
 
    thetaout=[thetaout1;thetaout2;thetaout3];
    
    