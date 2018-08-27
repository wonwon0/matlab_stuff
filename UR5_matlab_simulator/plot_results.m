clear
close all

% genere les graphs de resultats de l'experimentation 6DOF pour l'Article
% de l'ICRA2019

t_algo_TT_RR = sort([241 327 110.33 84.61 115 118 158 105 87 137 141 188 183 89 82]);
t_algo_TR_TR = sort([134 114 78.5 42 55 48 96 45 66 78 57 95 95 57 48]);
t_algo_SPR_D = sort([134 89 85.83 66 88 65 142 53 107 104 67 148 55 100 54]);
t_TR_TR_1st_obs = [15.4 11.9 15 15 35 12 13 15 14 32 35 23 13];
t_TR_TR_2nd_obs = [130.2 81.5 28 41 61 99 22 60 19 224 44 18 28];
               
% t_algo_TT_RR = sort([1.42043222 1.8509433962 1.2050899294 1.3178443487 1.3372093023 1.5324675325 1.196969697 1.5517241379 1.0038461538 1.2884012539 1.5962264151]);
% t_algo_TR_TR = sort([0.78978389 0.6452830189 0.8574237239 0.6541716422 0.6395348837 0.6233766234 0.7272727273 0.6650246305 0.7615384615 0.7335423197 0.6452830189]);
% t_algo_SPR_D = sort([0.78978389 0.5037735849 0.9374863468 1.0279840091 1.023255814 0.8441558442 1.0757575758 0.7832512315 1.2346153846 0.9780564263 0.758490566]);


%affichage graphiques
vec_zeros = zeros(length(t_algo_TT_RR));
t_algo_TT_RR_y = vec_zeros;
t_algo_TR_TR_y = vec_zeros;
t_algo_SPR_D_y = vec_zeros;


figure
scatter(1:length(t_algo_TT_RR) ,t_algo_TT_RR, '^r');
hold on
scatter(1:length(t_algo_TT_RR) ,t_algo_TR_TR, 'Ob');
hold on
scatter(1:length(t_algo_TT_RR) ,t_algo_SPR_D, 'Sg');

figure
bar(1:3 ,[t_algo_TT_RR;t_algo_TR_TR;t_algo_SPR_D], 'FaceColor',[0 .5 .5]);
















