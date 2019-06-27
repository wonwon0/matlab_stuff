function [normale_effecteur]=PointToEffector_v4(vect_norm_plan, jacob_pt, jacob_eff )
    %semble ne pas bien fonctionner pour les rotations
    vect_norm_plan=[vect_norm_plan,0 ,0, 0];
    K=jacob_pt/jacob_eff;
    normale_effecteur = K'*vect_norm_plan';
    normale_effecteur(abs(normale_effecteur)<0.0000000000000001)=0;
    normale_effecteur = normale_effecteur / norm(normale_effecteur);
    normale_effecteur(isnan(normale_effecteur))=0;
    normale_effecteur = normale_effecteur / norm(normale_effecteur);
    normale_effecteur = normale_effecteur';
end