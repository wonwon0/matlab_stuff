function [Limitation,alpha] = a_LimitationPlanes2_3D(theta,T)

global Slide1_Extension_delta;
global Slide1_Theta2_delta;
global Slide1_Theta3_delta;

global Slide2_Extension_delta;
global Slide2_Theta2_delta;
global Slide2_Theta3_delta;

    [DHa,DHb,DHal,DHth] = a_paramDH();
    [LimitAng] = a_LimitAng();

    [J, Jinv] = a_Jacobian(theta);
       
    Limitation = [];
    
    alpha=0;
    
    posActu = [T(1,4);T(2,4);T(3,4)];
    
    % Theta2 Limitation
        if theta(2) <= LimitAng(2,1)+Slide1_Theta2_delta;
            Limitation = [Limitation,[Jinv(2,1);Jinv(2,2);Jinv(2,3)]];
            alpha = max([alpha,(LimitAng(2,1)+Slide1_Theta2_delta-theta(2))/(Slide1_Theta2_delta-Slide2_Theta2_delta)]);
        elseif theta(2) >= LimitAng(2,2)-Slide1_Theta2_delta;
            Limitation = [Limitation,-[Jinv(2,1);Jinv(2,2);Jinv(2,3)]];
            alpha = max([alpha,-(LimitAng(2,2)-Slide1_Theta2_delta-theta(2))/(Slide1_Theta2_delta-Slide2_Theta2_delta)]);
        end
       
    % Theta3 Limitation
        if theta(3) <= LimitAng(3,1)+Slide2_Theta3_delta;
            Limitation = [Limitation,[Jinv(3,1);Jinv(3,2);Jinv(3,3)]];
            alpha = max([alpha,(LimitAng(3,1)+Slide1_Theta3_delta-theta(3))/(Slide1_Theta3_delta-Slide2_Theta3_delta)]);
        elseif theta(3) >= LimitAng(3,2)-Slide2_Theta3_delta;
            Limitation = [Limitation,-[Jinv(3,1);Jinv(3,2);Jinv(3,3)]];
            alpha = max([alpha,-(LimitAng(3,2)-Slide1_Theta3_delta-theta(3))/(Slide1_Theta3_delta-Slide2_Theta3_delta)]);
        end


  
        
        alpha = min([1,alpha]);
        alpha=max([0,alpha]);
        
        
        
       
