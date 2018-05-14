function [normale_effecteur]=PointToEffector_v6(vect_norm_plan, jacob_pt, jacob_eff)

    vect_norm_plan=[0, 0, 0, vect_norm_plan];
    K_trans=jacob_pt/jacob_eff;
    normale_effecteur_trans = K_trans'*vect_norm_plan';
    normale_effecteur_trans(abs(normale_effecteur_trans)<0.0000000000000001)=0;
    normale_effecteur_trans = normale_effecteur_trans / norm(normale_effecteur_trans);
    normale_effecteur_trans(isnan(normale_effecteur_trans))=0;
    normale_effecteur = [normale_effecteur_trans(4:6); normale_effecteur_trans(1:3)];
    normale_effecteur = normale_effecteur';
end