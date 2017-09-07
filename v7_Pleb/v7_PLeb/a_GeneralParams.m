function a_GeneralParams()


    % BASE SINGULARITY
        global Hard_cylindreInt; % Rayon du cylindre intérieur
                Hard_cylindreInt=0.02;

    % DISTANCE SLIDE 2        
        global Slide2_Theta2_delta; % Theta2 Slide2 distance 
                Slide2_Theta2_delta = 6*pi/180;

        global Slide2_Theta3_delta; % Theta2 Slide2 distance   
                Slide2_Theta3_delta = 6*pi/180;

        global Slide2_Extension_delta; % Extension Slide2
                Slide2_Extension_delta = 0.025;

        global Slide2_ProtectionZonePrismGap; % Distance hard et slide2
                Slide2_ProtectionZonePrismGap = 0.03;
            
          
    % DISTANCE SLIDE 1         
        global Slide1_Theta2_delta; % Ditance theta2 slide 1
                Slide1_Theta2_delta = 12*pi/180;

        global Slide1_Theta3_delta; % Ditance theta3 slide 1
                Slide1_Theta3_delta = 12*pi/180;

        global Slide1_Extension_delta; % Extension Slide1
                Slide1_Extension_delta = 0.075;

        global Slide1_ProtectionZonePrismGap; % Distance hard et slide1
                Slide1_ProtectionZonePrismGap = 0.06;     
        
    % MAX SPEED       
        global Cart_Velocity_Max; % Vitesse cart max
                Cart_Velocity_Max = 0.03;

        global Cart_VelocitySlowZone_Factor; % Ratio of full speed
                Cart_VelocitySlowZone_Factor = 0.5;

        global Ang_Velocity_Max; % Vitesse angulaire max
                Ang_Velocity_Max = 2*pi/180;

            
    % BASE CONE-CYLINDRE PROTECTION 
        global Base_H1;
        Base_H1 = -0.15;

        global Base_H2;
        Base_H2 = -0.05;

        global Base_H3;
        Base_H3 = 0.35;

        global Base_H4;
        Base_H4 = 0.50;

        global Base_Cyl1;
        Base_Cyl1 = 0.15;

        global Base_gapslide;
        Base_gapslide = 0.03;

        global Base_gapbeginslide;
        Base_gapbeginslide = 0.06;

            
end