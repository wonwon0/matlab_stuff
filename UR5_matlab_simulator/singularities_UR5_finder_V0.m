function [normales_effecteur, d_min] = singularities_UR5_finder_V0(theta_act, pose, dh, tol)
%fonction trouve les normales à l'effecteur qui représentent l'hyper-plan
%généré par deux des singularités du robot UR5, soit theta_3 = 0 et
%theta_5 = 0 ou pi.
normales_effecteur = [];
d_min = [];
theta_act = wrapToPi(theta_act);
jacob_eff = jacob_UR5_v1( theta_act, pose, dh);
if (abs(theta_act(5)) < tol)
    vec = -null(jacob_eff(:,[1,2,3,4,6])')';
    normales_effecteur = [normales_effecteur [vec(4:6) vec(1:3)]];
    d_min = [d_min abs(theta_act(5)) / tol];
elseif abs(theta_act(5)) > (pi - tol)
    vec = null(jacob_eff(:,[1,2,3,4,6])')';
    normales_effecteur = [normales_effecteur [vec(4:6) vec(1:3)]];
    d_min = [d_min abs(theta_act(5) - pi) / tol];
end