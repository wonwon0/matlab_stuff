function [v_output]=verifVitesse_ForceField(v_input,vec_norm,d_min)
    gaprepuls=0.08; %0.04 rien ne passe
    gainPos = 100;
    FmaxPos = 3.5;
    gainK = 400000;
    m=1;
    c=1000;
    Ts=0.01;
    Axe=v_input';
    v=Axe;
    Ftot=0;
    vmax=1;
    
    for i = 1:size(vec_norm,1)
        d = d_min(i);
        vecteur = vec_norm(i,:);
        deltax=0;
        
        %         if i==3
        %             if vecteur'*LimitS(i,5:6)' > 0
        %                 test=1;
        %             else
        %                 test=2;
        %             end
        %         end
            
        
        [ k ] = gain_force( gainK, d, 0.02,gaprepuls );
        deltax = gaprepuls;
       %abs(gaprepuls-d);

        F=(k*(gaprepuls-d)*vec_norm(i,:)-c*v')/d;
%        F = deltax * gainK * vecteur;
%        F = deltax * gainK * vecteur+10*v';



%         
%         if (i==1 || i==4) && x(1) < 0
%             F=0;
%         end
%         if (i==3 || i==6) && x(1) > 0
%             F=0;
%         end 
        
        
        F;
        Ftot=Ftot+F;
    end
    if isempty(vec_norm)
        a=0;
    else
        a = (Ftot')/m;
    end
    v = v + a*Ts;
    if norm(v) > 1
        v = v/norm(v);
%         x = x + v*Ts;
%     else
%         x = x + v*Ts + 0.5*a*Ts^2;  
    end


    v_output=v';
        
        
end