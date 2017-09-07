function vit1 = a_VitCartMax(vit,Cart_Velocity_Max)

    % Limiter la vitesse max
    vitX = vit(1);
    vitY = vit(2);
    vitZ = vit(3);
    vitnorm = sqrt(vitX^2+vitY^2+vitZ^2);
    
    vitmax = Cart_Velocity_Max;
    if vitnorm > vitmax
        vitX=vitX*vitmax/vitnorm;
        vitY=vitY*vitmax/vitnorm;
        vitZ=vitZ*vitmax/vitnorm;
    end
    
    vit1=[vitX;vitY;vitZ];
    

end
