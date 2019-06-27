function [v_output]=verifVitesse_ForceField(v_input,vec_norm,d_min, min_distance)
    d_min = d_min / 1000;
    gaprepuls=min_distance/1000; %0.04 rien ne passe
    gainPos = 10;
    gainK = 10000;
    m=1;
    c=0.01;
    Ts=0.01;
    Axe=v_input';
    v=Axe;
    Ftot=0;
    vmax=1;
    
    for i = 1:size(vec_norm,1)
        d = d_min(i);
        vecteur = vec_norm(i,:);
        deltax=0;
        [ k ] = gain_force( gainK, d, 0.02 + gaprepuls,gaprepuls );
        deltax = gaprepuls;

        F=((k*(gaprepuls-d)*vec_norm(i,:)-c*v')/d).*[1 1 1 0.01 0.01 0.01];
        Ftot=Ftot+F;
    end
    if isempty(vec_norm)
        a=0;
    else
        a = (Ftot')/m;
    end
    v = v + a*Ts ;
    if norm(v) > 1
        v = v/norm(v);
    end


    v_output=v';
        
        
end