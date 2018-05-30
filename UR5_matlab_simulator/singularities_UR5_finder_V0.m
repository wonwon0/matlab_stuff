function [normales_effecteur] = singularities_UR5_finder_V0(theta_act, pose, dh, tol)
%fonction trouve les normales à l'effecteur qui représentent l'hyper-plan
%généré par deux des singularités du robot UR5, soit theta_3 = 0 et
%theta_5 = 0 ou pi.
normales_effecteur = [];
theta_act = wrapToPi(theta_act);
jacob_eff = jacob_UR5_v1( theta_act, pose, dh);
% if abs(theta_act(3)) < tol*10
%     normales_effecteur = [normales_effecteur -null(jacob_eff(:,[1,2,4,5,6])')'];
% end

if (abs(theta_act(5)) < tol) || (abs(theta_act(5)) > (pi - tol))
    normales_effecteur = [normales_effecteur -null(jacob_eff(:,[1,2,3,4,6])')'];
end
if ~isempty(normales_effecteur)
    normales_effecteur = [normales_effecteur(: , 4:6) normales_effecteur(: , 1:3)];
end