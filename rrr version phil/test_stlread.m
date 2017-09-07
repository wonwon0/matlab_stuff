figure
clear
close all
pose_init=[0.3,0.58,0.3];
L=[0.2755 0.290 0.376 0.0098];
theta_init=cin_inv(pose_init,[0 -pi/2 -pi/2],L);
P1=[0 0 L(1)];
P2=[L(2)*cos(theta_init(2)).*cos(theta_init(1)) L(2)*cos(theta_init(2)).*sin(theta_init(1)) L(2)*sin(theta_init(2))]+P1;
P3=cin_dir(theta_init, [0.2755 0.290 0 0.0098]);
P4=cin_dir(theta_init, [0.2755 0.290 0.2 0.0098]);
P5=cin_dir(theta_init, L);
Q1=[cos(theta_init(1)) -sin(theta_init(1))*cos(0) sin(0)*sin(theta_init(1)); sin(theta_init(1)) cos(0)*cos(theta_init(1)) -sin(0)*cos(theta_init(1)); 0 sin(0) cos(0)];
Q2=[1 0 0; 0 cos(theta_init(2)+pi/2) -sin(theta_init(2)+pi/2); 0 sin(theta_init(2)+pi/2) cos(theta_init(2)+pi/2)];
Q3=[1 0 0; 0 cos(theta_init(3)) -sin(theta_init(3)); 0 sin(theta_init(3)) cos(theta_init(3))];

%import de la base
[v1, f1, n1, c1, stltitle] = stlread('M0_bin.STL', 0);
[v1, f1]=patchslim(v1, f1);
v1=[v1(:,1)-0.052 v1(:,2) v1(:,3)-0.05];
v1=[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]*[cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1]*v1';
v1=v1';
v1=[v1(:,1) v1(:,2) v1(:,3)+0.15656];


%import du joint 1
[v2, f2, n, c, stltitle] = stlread('M1_bin.STL', 0);
[v2, f2]=patchslim(v2, f2);
v2=[v2(:,1)-0.18737 v2(:,2)-0.04276 v2(:,3)-0.04181];
v2=[cos(pi/2) 0 sin(pi/2); 0 1 0; -sin(pi/2) 0 -cos(pi/2)]*v2';
v2=[cos(pi/2) -sin(pi/2) 0; sin(pi/2) cos(pi/2) 0; 0 0 1]*v2;
v2=(Q1*v2)';
v2=[v2(:,1) v2(:,2) v2(:,3)+0.15656];

%import de la membrure 1
[v3, f3, n, c, stltitle] = stlread('M2_bin.STL', 0);
[v3, f3]=patchslim(v3, f3);
v3=[v3(:,1)-0.04120 v3(:,2)-0.04120 v3(:,3)-0.11257];
v3=v3';
v3=[cos(-pi/2) 0 sin(-pi/2); 0 1 0; -sin(-pi/2) 0 -cos(-pi/2)]*v3;
v3=[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]*v3;
v3=[cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1]*v3;
v3=(Q2*v3);


v3=(Q1*v3)';
v3=[v3(:,1) v3(:,2) v3(:,3)+0.275];

%import de la membrure 2
[v4, f4, n, c, stltitle] = stlread('M4_bin.STL', 0);
[v4, f4]=patchslim(v4, f4);
v4=v4/1000;

v4=[v4(:,1)-0.0098 v4(:,2)-0.173 v4(:,3)-0.6471];
v4=v4*[1 0 0; 0 cos(40*pi/180) -sin(40*pi/180); 0 sin(40*pi/180) cos(40*pi/180)];

v4=[1 0 0; 0 cos(-pi) -sin(-pi); 0 sin(-pi) cos(-pi)]*v4';
v4=(Q3*v4);
v4=(Q2*v4);
v4=(Q1*v4)';

v4=[v4(:,1)+P3(1) v4(:,2)+P3(2) v4(:,3)+P3(3)];

%import de la main fermée
[v5, f5, n, c, stltitle] = stlread('M5_bin.STL', 0);
[v5, f5]=patchslim(v5, f5);
v5=v5/1000;
v5=v5*[1 0 0; 0 cos(40*pi/180) -sin(40*pi/180); 0 sin(40*pi/180) cos(40*pi/180)];
v5=[v5(:,1)-0.01 v5(:,2)-0.215 v5(:,3)-0.311];
v5=[1 0 0; 0 cos(-pi) -sin(-pi); 0 sin(-pi) cos(-pi)]*v5';
v5=(Q3*v5);
v5=(Q2*v5);
v5=(Q1*v5)';
v5=[v5(:,1)+P4(1) v5(:,2)+P4(2) v5(:,3)+P4(3)];


h1=patch('Faces',f1,'Vertices',v1,'FaceVertexCData',c);
set(h1, 'facec', [0 0.6 1]);
set(h1, 'EdgeColor','b');
hold on
h2=patch('Faces',f2,'Vertices',v2,'FaceVertexCData',c);
set(h2, 'facec', [0 0.6 1]);
set(h2, 'EdgeColor','b');
hold on

h3=patch('Faces',f3,'Vertices',v3,'FaceVertexCData',c);
set(h3, 'facec', [0 0.6 1]);
set(h3, 'EdgeColor','b');
hold on

h4=patch('Faces',f4,'Vertices',v4,'FaceVertexCData',c);
set(h4, 'facec', [0 0.6 1]);
set(h4, 'EdgeColor','b');
hold on

h5=patch('Faces',f5,'Vertices',v5,'FaceVertexCData',c);
set(h5, 'facec', [0 0.6 1]);
set(h5, 'EdgeColor','b');
hold on

plot3(P3(1),P3(2),P3(3),'X','color','r');
hold on
plot3(P5(1),P5(2),P5(3),'O','color','r');
xlabel('x')
ylabel('y')
zlabel('z')
view(90,0);