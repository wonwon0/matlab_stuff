close all
clear all
clc
temps_test1=[ 48
69
50
42
83
39
36
34
31
31
51
48
39];
temps_test2=[92
94
82
80
185
88
74
50
65
55
80
88
92];
temps_test3=[54
63
54
42
63
41
48
38
33
34
50
41
41
];
temps_test4=[84
65
76.06
96
189
88
71
86
70
73
86
91
138
];
erreur_test3=-1*[0;0;0;0;4;2;2;3;2;0;2;4;6];
erreur_test4=-1*[2;2;10;20;30;4;0;8;11;14;3;3;31];
Part=[temps_test1 temps_test2 temps_test3 temps_test4 zeros(13,1) erreur_test3 zeros(13,1) erreur_test4];


Exp1_Full_Temps_Algo = Part(:,1);
Exp1_Full_Temps_NoAlgo = Part(:,2);
% hist(Exp1_Full_Temps_NoAlgo)
Exp2_Div_Temps_Algo = Part(:,3);
Exp2_Div_Temps_NoAlgo = Part(:,4);

data = [Exp1_Full_Temps_Algo,Exp1_Full_Temps_NoAlgo,Exp2_Div_Temps_Algo,Exp2_Div_Temps_NoAlgo];
% ranovatbl = anova(data)



Exp2_Div_dep_Algo = Part(:,5)+Part(:,6);
Exp2_Div_dep_NoAlgo = Part(:,7)+Part(:,8);





% [t,p]=ttest(Exp1_Full_Temps_Algo,Exp1_Full_Temps_NoAlgo)
% [t,p]=ttest(Exp2_Div_Temps_Algo,Exp2_Div_Temps_NoAlgo)


% plot(Exp1_Full_Temps_Algo,1,'xb');hold on;
% plot(Exp1_Full_Temps_NoAlgo,1,'or')
plot(Exp2_Div_Temps_Algo,-Exp2_Div_dep_Algo,'xb');hold on;
plot(Exp2_Div_Temps_NoAlgo,-Exp2_Div_dep_NoAlgo,'or');

avr = mean(Part(:,1));
avr = mean(Part(:,2));
avr = mean(Part(:,3));
avr = mean(Part(:,4));
varr = std(Part(:,1));
varr = std(Part(:,2));
varr = std(Part(:,3));
varr = std(Part(:,4));

mean(Part(:,5)+Part(:,6));
mean(Part(:,7)+Part(:,8));

% [t,p]=ttest(Part(:,1)',Part(:,2)')
p1 = mwwtest(Part(:,1)',Part(:,2)')
% p = signrank(Part(:,1)',Part(:,2)')

% [t,p]=ttest(Part(:,3)',Part(:,4)')
p2 = mwwtest(Part(:,3)',Part(:,4)')


p3 = mwwtest(Part(:,3)'-Part(:,1)',Part(:,4)'-Part(:,2)')

p4 = mwwtest((-Part(:,5)-Part(:,6))',(-Part(:,7)-Part(:,8))')

% p = ranksum(Part(:,1),Part(:,2)) %wincoxon
% p = ranksum(Part(:,3),Part(:,4)) %wincoxon
% p = ranksum(-Part(:,5)-Part(:,6),-Part(:,7)-Part(:,8)) %wincoxon

% % p = mwwtest(Part(:,1)',Part(:,2)')

% Plot Test
figure(2) 
plot(Part(:,1),[0.7 1 1.3 1 1 0.7 1 1 1.3 0.7 1 1.3 1.3],'xb','LineWidth',2,'MarkerSize',10);
hold on
Part2_temp=Part(:,2);
Part2_temp(5)=190;
BreakXAxis(Part2_temp,[0.7 1 1 1.3 1 0.7 1 0.7 1 1 0.7 1.3 1.3],100,178,1);
hold on;
set(gca,'YTick',[])
plot(mean(Part(:,1)),[2.5],'xb','LineWidth',3,'MarkerSize',20);
plot(mean(Part(:,2)),[2.5],'or','LineWidth',3,'MarkerSize',20);
ylim([0 3])


% figure(2)
% hist(Part(:,1),10)
% h = findobj(gca,'Type','patch');
% set(h,'FaceColor','r','EdgeColor','w','facealpha',0.75)
% hold on;
% hist(Part(:,2),10)
% h1 = findobj(gca,'Type','patch');
% set(h1,'facealpha',0.75);

figure(3)
plot(Part(:,3),-Part(:,5)-Part(:,6),'xb','LineWidth',2,'MarkerSize',15);
hold on;
plot(Part(:,4),-Part(:,7)-Part(:,8),'or','LineWidth',2,'MarkerSize',15);
plot(mean(Part(:,3)),mean(-Part(:,5)-Part(:,6)),'xb','LineWidth',4,'MarkerSize',30)
plot(mean(Part(:,4)),mean(-Part(:,7)-Part(:,8)),'or','LineWidth',3,'MarkerSize',30)
xlim([35 200])
ylim([-0.5 32])
set(gca,'FontSize', 25);
% mean(Part(:,1))/mean(Part(:,2))
(mean(Part(:,2))-mean(Part(:,1)))/mean(Part(:,2))

% mean(Part(:,3))/mean(Part(:,4))
(mean(Part(:,4))-mean(Part(:,3)))/mean(Part(:,4))

% figure(4)
% hist(Part(:,3),3)
% h = findobj(gca,'Type','patch');
% set(h,'FaceColor','r','EdgeColor','w','facealpha',0.75)
% hold on;
% hist(Part(:,4),7)
% h1 = findobj(gca,'Type','patch');
% set(h1,'facealpha',0.75);


