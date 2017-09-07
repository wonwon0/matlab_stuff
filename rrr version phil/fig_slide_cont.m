
close all
clear
addpath('U:\matlab\rrr version phil\geom3d\geom3d')
figure
axis([0 2 0 2 0 2]);
p0 = [0 0 0];
v1 = [0 0 1];
v2 = [0 1 0];
plane1 = [p0 v1 v2];
drawPlane3d(plane1)
hold on
v3 = [1 0 0.7];
v4 = [0 1 0];
plane2 = [p0 v3 v4];
drawPlane3d(plane2)
hold on
p1 = [0 2 0];
v5 = [0 0 1];
v6 = [1 0 0];
plane3 = [p1 v5 v6];
drawPlane3d(plane3)
axis([-0.1 2.1 -0.1 2.1 -0.1 2.1]);
set(gcf, 'renderer', 'zbuffer');
hold on
vec=[0.75 1 1.5]-[0.5 0.7 1.5]-[0.75 1 1.5];
vec=vec/norm(vec);
vectarrow([0.75 1 1.5],[0.75 1 1.5]+vec,'b')
hold on


perp1=cross(v1,v2)/norm(cross(v1,v2));
perp2=cross(v3,v4)/norm(cross(v3,v4));
perp3=cross(v5,v6)/norm(cross(v5,v6));
vect1=vec-((vec*perp1')*perp1')';
vect2=vec-((vec*perp2')*perp2')';
vect3=vec-((vec*perp3')*perp3')';
point1 = projPointOnPlane([0.75 1 1.5], [p0 v1 v2]);
point2 = projPointOnPlane([0.75 1 1.5], [p0 v3 v4]);
point3 = projPointOnPlane([0.75 1 1.5], [p1 v5 v6]);
vectarrow(point1,point1+vect1,'r')
hold on
vectarrow(point2,point2+vect2,'r')
hold on
%vectarrow(point3,point3+vect3,'r')
hold on
line1 = intersectPlanes(plane1, plane2);
point4 = projPointOnLine3d(point1, line1);
line2 = intersectPlanes(plane1, plane3);
point5 = projPointOnLine3d(point1, line2);
line3 = intersectPlanes(plane2, plane3);
point6 = projPointOnLine3d(point3, line3);

com1=cross(perp1,perp2)/norm(cross(perp1,perp2));
com2=cross(perp1,perp3)/norm(cross(perp1,perp3));
com3=cross(perp2,perp3)/norm(cross(perp2,perp3));


vect4=((vec*com1')*com1')';
vect5=((vec*com2')*com2')';
vect6=((vec*com3')*com3')';

vectarrow(point4,point4+vect4,[0 1 0])
hold on
%vectarrow(point5,point5+vect5,[1 0 0])
hold on
%vectarrow(point6,point6+vect6,[1 0 0])
hold on




figure
axis([0 1.5 0 1.5 0 1.5]);
drawPlane3d(plane1)

set(gcf, 'renderer', 'zbuffer');
vec=[0.75 1 1.5]-[0.5 0.7 1.5]-[0.75 1 1];
vec=vec/norm(vec);
hold on
drawPlane3d(plane2)
axis([-0.5 1.5 -0.5 1.5 -0.5 1.5]);
hold on
vectarrow(-vec,-vec+vec,'b')
hold on
vect1=vec-((vec*perp1')*perp1')';
vect2=vec-((vec*perp2')*perp2')';
point1 = projPointOnPlane(-vec, [p0 v1 v2]);
point2 = projPointOnPlane(-vec, [p0 v3 v4]);
vectarrow(point1,point1+vect1,'r')
hold on
vectarrow(point2,point2+vect2,'r')

%%
close all
temp_test1=[ 48
69.44
50
41.62
83.48
39
36
34.52
31.64
31
51
48
39];
temp_test2=[91.65
93.93
81.57
80
185
88.41
74
50
65
55
80
88
92];
temp_test3=[53.77
63
53.66
42.14
63
41
48
38.15
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
erreur_test1=[0;1;1;0;0;0;0;0;0;0;1;1;1];
erreur_test2=[1;2;1;3;0;0;0;1;1;0;0;2;2];
erreur_test3=[0;0;0;0;4;2;2;3;2;0;2;4;6];
erreur_test4=[2;2;10;20;30;4;8;11;14;3;3;31];

x=[1;2;3;4;5;6;7;8;9;10;11;12;13];


figure
scatter(x,temp_test1);
hold on
scatter(x,temp_test2);

[ax, p1, p2]=plotyy(x,temp_test2,x,erreur_test2);
delete(p1)
delete(p2)

hold(ax(1),'on')
scatter(x,temp_test1);
hold on
scatter(x,temp_test2);
hold(ax(2),'on')
scatter(ax(2),x,erreur_test1);
hold on
scatter(ax(2),x,erreur_test2);
















