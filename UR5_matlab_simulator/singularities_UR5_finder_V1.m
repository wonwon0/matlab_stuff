function [normales_effecteur, d_min] = singularities_UR5_finder_V1(theta_act, pose, dh, tol)
%fonction trouve les normales à l'effecteur qui représentent l'hyper-plan
%généré par deux des singularités du robot UR5, soit theta_3 = 0 et
%theta_5 = 0 ou pi.
normales_effecteur = [];
d_min = [];
theta_act = wrapToPi(theta_act);
jacob_eff = jacob_UR5_v1( theta_act, pose, dh);
dh_rep_6 = dh;
dh_rep_6.a = dh.a .* [1 1 1 1 0 0];
dh_rep_6.b = dh.b .* [1 1 1 1 0 0];
dh_rep_6.alpha = dh.alpha .* [1 1 1 1 0 0];
jacob_repere_6 = jacob_UR5_v1( theta_act, pose, dh_rep_6);
if ((theta_act(5) < tol) && (theta_act(5) > 0)) || (theta_act(5) < (-pi + tol))
    forbiden_ang_dir = [ 0 0 0 0 1 0]';
    if abs(theta_act(5)) > tol
        disp('cas1')
        d_min = [d_min abs(theta_act(5) + pi) / tol];
    else
        disp('cas2')
        d_min = [d_min abs(theta_act(5)) / tol];
    end
    forbiden_dir_rep_6 = jacob_repere_6 * forbiden_ang_dir;
    forbiden_dir_rep_6_t = forbiden_dir_rep_6(1:3);
    forbiden_dir_rep_6_t(abs(forbiden_dir_rep_6_t)<0.01) = 0;
    forbiden_dir_rep_6(1:3) = forbiden_dir_rep_6_t;
    forbiden_dir_rep_6(abs(forbiden_dir_rep_6)<0.00000001) = 0;
    forbiden_dir = PointToEffector_v6(forbiden_dir_rep_6', jacob_repere_6, jacob_eff);
    normales_effecteur = [normales_effecteur forbiden_dir];
elseif (theta_act(5) > (pi - tol)) || ((theta_act(5) > -tol) && (theta_act(5) < 0))
    forbiden_ang_dir = [ 0 0 0 0 -1 0]';
    if abs(theta_act(5)) > tol
        disp('cas3')
        
        d_min = [d_min abs(theta_act(5) - pi) / tol];
    else
        disp('cas4')
        d_min = [d_min abs(theta_act(5)) / tol];
    end
    forbiden_dir_rep_6 = jacob_repere_6 * forbiden_ang_dir;
        forbiden_dir_rep_6_t = forbiden_dir_rep_6(1:3);
    forbiden_dir_rep_6_t(abs(forbiden_dir_rep_6_t)<0.01) = 0;
    forbiden_dir_rep_6(1:3) = forbiden_dir_rep_6_t;
    forbiden_dir_rep_6(abs(forbiden_dir_rep_6)<0.00000001) = 0;
    forbiden_dir = PointToEffector_v6(forbiden_dir_rep_6', jacob_repere_6, jacob_eff);
    normales_effecteur = [normales_effecteur forbiden_dir];
end