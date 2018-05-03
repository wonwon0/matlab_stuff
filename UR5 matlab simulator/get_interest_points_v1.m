function [points, jacobians] = get_interest_points_v1(theta, dh_eff)
%fonctionn donnant les points d'intéret du robot UR5.
%pour ce faire, on modifie les parametres dh en appliquant un masque qui
%modifie la longueure des membrures en fonction de l'emplacment du point
%d'interet. Se referer au diagrame du robot pour mieux comprendre.
mask_a = [1 1 1 1 1 1; %effecteur
    1 1 1 1 1 0; % repere 6
    1 1 1 1 0 0; % repere 5
    1 1 1 0 0 0; % repere 4
    1 1 0 0 0 0; % coude entre le repere 3 et 4
    1 1 0 0 0 0; % repere 3
    1 0 0 0 0 0; % coude entre le repere 2 et 3
    1 0 0 0 0 0]; % repere 2
mask_b = [1 1 1 1 1 1;
    1 1 1 1 1 0;
    1 1 1 1 0 0;
    1 1 1 0 0 0;
    1 1 1 0 0 0;
    1 1 0 0 0 0;
    1 1 0 0 0 0;
    1 0 0 0 0 0];
dh_interest = dh_eff;
for i =1:8
    dh_interest.a = dh_eff.a .* mask_a(i,:)';
    dh_interest.b = dh_eff.b .* mask_b(i,:)';
    dh_interest.alpha = dh_eff.alpha .* mask_b(i,:)';
    dh_interest.theta = dh_eff.theta .* mask_b(i,:)';
    [Pose, ~, ~] = cin_dir_6ddl_v3( theta, dh_interest);
    pose_cart = Pose(1:3, 4);
    points(:,i) = pose_cart;
    jacobians{i} = jacob_UR5_v1( theta, pose_cart, dh_interest);
    
end
end

