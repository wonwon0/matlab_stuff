clear
close all

% genere les graphs de resultats de l'experimentation 6DOF pour l'Article
% de l'ICRA2019

% t_algo_TT_RR = [241 327 110.33 84.61 114 118 158 105 87];
% t_algo_TR_TR = [134 114 78.5 42 55 48 96 45 66]';       
% t_algo_SPR_D = [134 89 85.8 66 88 65 142 53 107]';      
% t_TR_TR_1st_obs = [20 22 15.4 11.9 15 15 35 12 13]';    
% t_TR_TR_2nd_obs = [140 110 130 81.5 28 41 61 99 22]';
%                
t_algo_TT_RR = sort([1.42043222 1.8509433962 1.2050899294 1.3178443487 1.3372093023 1.5324675325 1.196969697 1.5517241379 1.0038461538 1.2884012539 1.5962264151]);
t_algo_TR_TR = sort([0.78978389 0.6452830189 0.8574237239 0.6541716422 0.6395348837 0.6233766234 0.7272727273 0.6650246305 0.7615384615 0.7335423197 0.6452830189]);
t_algo_SPR_D = sort([0.78978389 0.5037735849 0.9374863468 1.0279840091 1.023255814 0.8441558442 1.0757575758 0.7832512315 1.2346153846 0.9780564263 0.758490566]);


%affichage graphiques
vec_zeros = zeros(length(t_algo_TT_RR));
t_algo_TT_RR_y = vec_zeros;
t_algo_TR_TR_y = vec_zeros;
t_algo_SPR_D_y = vec_zeros;
t_algo_TR_TR_y([3,5,7,8,9]) = 1;


figure
plot(t_algo_TT_RR ,t_algo_TT_RR_y, 'Ob','LineWidth',3);
hold on
plot(t_algo_TR_TR, t_algo_TR_TR_y, 'Xr','LineWidth',3);
hold on
plot(t_algo_SPR_D, t_algo_SPR_D_y, 'Sg','LineWidth',3);
ylim([-1.5,1.5])


















