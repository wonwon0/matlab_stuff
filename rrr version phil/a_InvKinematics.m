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

    D1=DHb(1);
    D2=DHa(2);
    D3=DHa(3);
    e2 = DHb(3);
    

    thetainRobot = theta;
    
    Erreur=[0;0;0;0];
    
    pxdes = posDes(1);
    pydes = posDes(2);
    pzdes = posDes(3);

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
        thetainRobot=[thetainRobot(1);thetainRobot(2);thetainRobot(3)];
        DeltaTheta = theta_IK - thetainRobot;
		for h=1:3
			if (DeltaTheta(h) > 10)
				flagErreur = 1;
            end
        end
        
        thetaout1 = theta_IK(1);
        thetaout2 = theta_IK(2);
        thetaout3 = theta_IK(3);
 
    thetaout=SolutionF;
    SolutionF
    
    