function Limitation = a_LimitationPlanes2_2D(theta,T)

global Slide2_Extension_delta;
global Slide2_Theta1_delta;
global Slide2_Theta2_delta;


    [DHa,DHb,DHal,DHth] = a_paramDH();
    [LimitAng] = a_LimitAng();

    L1=DHa(1);
    L2=DHa(2);
    
    [J, Jinv] = a_Jacobian(theta);
       
    Limitation = [];
    

    % Theta1 Limitation
        if theta(1) <= LimitAng(1,1)+Slide2_Theta1_delta;
            Limitation = [Limitation,[Jinv(1,1);Jinv(1,2);0]];
        elseif theta(1) >= LimitAng(1,2)-Slide2_Theta1_delta;
            Limitation = -[Limitation,[Jinv(1,1);Jinv(1,2);0]];
        end
       
    % Theta2 Limitation
        if theta(2) <= LimitAng(2,1)+Slide2_Theta2_delta;
            Limitation = [Limitation,[Jinv(2,1);Jinv(2,2);0]];
        elseif theta(2) >= LimitAng(2,2)-Slide2_Theta2_delta;
            Limitation = -[Limitation,[Jinv(2,1);Jinv(2,2);0]];
        end
        
    % Maximum reach limitation
        if sqrt(T(1,4)^2+T(2,4)^2) >= (L1+L2)-Slide2_Extension_delta
            Limitation = -[Limitation,[T(1,4);T(2,4);0]];
        end
        
        
    % Protection zones - prisms
        [T,T1,T2] = a_CinDirecte([theta(1);theta(2)]);
        [Prisms, Cylinders, Spheres, Cones] = a_ProtectionZoneDef();
        
        posActu = [T(1,4);T(2,4);0];
        
        
        global Slide2_ProtectionZonePrismGap;
        
        [Limitation, inside] = a_ProtectionZonePrisms2D(posActu, Prisms, Slide2_ProtectionZonePrismGap, Limitation);

        
        global active_elbow
        if active_elbow==1
            [Limitation, inside, minv] = a_ProtectionZonePrisms2D_elbow([T1(1,4);T1(2,4);0], theta, Prisms, Slide2_ProtectionZonePrismGap, Limitation);
        end
        
        
        
        
        
        
        
        
        
        
        
        
        
    
        
        
        
        
        