function Limitation = a_LimitationPlanes2_3D(theta,T)

global Slide2_Extension_delta;
global Slide2_Theta2_delta;
global Slide2_Theta3_delta;

    [DHa,DHb,DHal,DHth] = a_paramDH();
    [LimitAng] = a_LimitAng();

    [J, Jinv] = a_Jacobian(theta);
       
    Limitation = [];
    
    posActu = [T(1,4);T(2,4);T(3,4)];
    
    % Theta2 Limitation
        if theta(2) <= LimitAng(2,1)+Slide2_Theta2_delta;
            Limitation = [Limitation,[Jinv(2,1);Jinv(2,2);Jinv(2,3)]];
        elseif theta(2) >= LimitAng(2,2)-Slide2_Theta2_delta;
            Limitation = [Limitation,-[Jinv(2,1);Jinv(2,2);Jinv(2,3)]];
        end
       
    % Theta3 Limitation
        if theta(3) <= LimitAng(3,1)+Slide2_Theta3_delta;
            Limitation = [Limitation,[Jinv(3,1);Jinv(3,2);Jinv(3,3)]];
        elseif theta(3) >= LimitAng(3,2)-Slide2_Theta3_delta;
            Limitation = [Limitation,-[Jinv(3,1);Jinv(3,2);Jinv(3,3)]];
        end
        
        
