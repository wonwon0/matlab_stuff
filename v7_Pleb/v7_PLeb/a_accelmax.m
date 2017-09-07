function vit2 = a_accelmax(vit, vitm1, amax, dmax, dt)


acc = ((vit)-(vitm1)) / dt; %calcul de lacceleration

vit2 = vit;%Sortie

if (acc >=0 && vitm1 >= 0) %Accel
    if (acc > amax)
        vit2 = vitm1+amax*dt;
    end
elseif (acc <=0 && vitm1 <= 0) %Accel
    if (acc < -amax)
        vit2 = vitm1-amax*dt;
    end
elseif (acc >=0 && vitm1 < 0)%Decel
    if (acc > dmax)
        vit2 = vitm1+dmax*dt;
    end
elseif (acc <=0 && vitm1 > 0) %Decel
    if (acc < -dmax)
        vit2 = vitm1-dmax*dt;
    end  
end









