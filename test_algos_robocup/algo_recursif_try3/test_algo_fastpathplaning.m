clear
close all
addpath('U:\matlab\rrr version phil\geom3d\geom3d')
addpath('U:\matlab\rrr version phil v2\joystick soft 2')
pose_robot = [0, 0];
pose_target = [7000, 7000];
path.start = pose_robot;
path.goal = pose_target;
path.points  = [ pose_robot; pose_target];
depth = 0;
%load('pose_obstacle.mat')
n_obs = 10;
dia_obs = 300;
pose_obstacle_x = rand(n_obs,1)*5000 + 600;
pose_obstacle_y = pose_obstacle_x +(rand(n_obs,1)-0.5)*2 * 2000;
pose_obstacle = [pose_obstacle_x pose_obstacle_y];
new_target = [];
fig=figure(1);
set(fig, 'renderer', 'OpenGL');
set(fig, 'Units','normalized','position',[0 1 0.8 0.8])

datas = [pose_target; pose_obstacle];

obs = plot(datas(:,1), datas(:,2), 'Ob', 'LineWidth',4);
hold on
h1 = plot(pose_robot(1),pose_robot(2),'O', 'LineWidth', 7, 'Color', [0.5 0.4 1]);



axis equal
hold on
h2 = line([pose_robot(1), pose_target(1)], [pose_robot(2), pose_target(2)],'Color','b','LineWidth',2);
hold on
f4 = plot(path.points(:,1), path.points(:,2),'Or');
hold on
line_coords_x = [];
line_coords_y =[];
for i = 1:size(pose_obstacle,1)
    d(i) = circle2D(pose_obstacle(i,:),dia_obs,[0 0 1], 3);
    hold on
end
for i = 1:size(path.points,1)-1
    line_coords_x = [line_coords_x path.points(i,1) path.points(i+1,1), NaN];
    line_coords_y = [line_coords_y path.points(i,2) path.points(i+1,2), NaN];
end
h3 = line(line_coords_x, line_coords_y,'Color','r','LineWidth',2);
hold off
drawnow
run = 1;
display('press a to start')
% while true
%    [pos, but] = mat_joy(0);
%    
%     if but(2)==1
%         run=1;
%         break
%     end
% end
vit_obstacle_r = (rand(n_obs,1)-0.5)*2 * 20;

count = 0;
while run
    count = count+0.05;
    if norm(path.start - path.goal) < 10
        path.start = [0 0];
        pose_obstacle_x = rand(n_obs,1)*5000 + 600;
        pose_obstacle_y = pose_obstacle_x +(rand(n_obs,1)-0.5)*2 * 2000;
        vit_obstacle_r = (rand(n_obs,1)-0.5)*2 * 20;
        
        pose_obstacle = [pose_obstacle_x pose_obstacle_y];
        fig=figure(1);
        set(fig, 'renderer', 'OpenGL');
        set(fig, 'Units','normalized','position',[0 1 0.8 0.8])

        datas = [path.goal; pose_obstacle];

        obs = plot(datas(:,1), datas(:,2), 'Ob', 'LineWidth',4);
        hold on
        h1 = plot(path.start(1),path.start(2),'O', 'LineWidth', 7, 'Color', [0.5 0.4 1]);



        axis equal
        hold on
        h2 = line([path.start(1), path.goal(1)], [path.start(2), path.goal(2)],'Color','b','LineWidth',2);
        hold on
        f4 = plot(path.points(:,1), path.points(:,2),'Or');
        hold on
        line_coords_x = [];
        line_coords_y =[];
        for i = 1:size(pose_obstacle,1)
            d(i) = circle2D(pose_obstacle(i,:),dia_obs,[0 0 1], 3);
            hold on
        end
        for i = 1:size(path.points,1)-1
            line_coords_x = [line_coords_x path.points(i,1) path.points(i+1,1), NaN];
            line_coords_y = [line_coords_y path.points(i,2) path.points(i+1,2), NaN];
        end
        h3 = line(line_coords_x, line_coords_y,'Color','r','LineWidth',2);
        hold off
        drawnow
    end
%     [pos, but] = mat_joy(0);
%     a=pos;
%     if (a(1)==0 && a(2)==0)
%         dir=[0 0];
%     elseif sqrt((double(a(1)))^2+(double(a(2)))^2)<1
%         dir=[double(a(1)),-double(a(2))];
%     else
%         dir=[double(a(1)),-double(a(2))]/sqrt((double(a(1)))^2+(double(a(2)))^2)^2;
%     end
    
    path.start = path.start + 20*(path.points(2,:) - path.start)/norm(path.points(2,:) - path.start);
    robot_speed = 20*(path.points(2,:) - path.start)/norm(path.points(2,:) - path.start);
    path.goal = [7000 7000];
    path.points = [path.start;path.goal];
    h1.XData = path.start(1);
    h1.YData = path.start(2);
    vit_obstavle_x_y =[];
    for i=1:length(d)
        pose_obstacle(i,:) = pose_obstacle(i,:) + [vit_obstacle_r(i) * cos(count), vit_obstacle_r(i) * sin(count)];
        obs.XData(i+1) = pose_obstacle(i,1);
        obs.YData(i+1) = pose_obstacle(i,2);
        vit_obstavle_x_y = [vit_obstavle_x_y; vit_obstacle_r(i) * cos(count) vit_obstacle_r(i) * sin(count)];
        d(i).XData = d(i).XData + vit_obstavle_x_y(i, 1);
        d(i).YData = d(i).YData + vit_obstavle_x_y(i, 2);
    end
    tic
    path = FastPathPlaning_2(pose_obstacle,vit_obstavle_x_y , path, robot_speed, depth, NaN, 1 );
    toc

    %path_len1 = Path_length( path );
    
    line_coords_x = [];
    line_coords_y =[];
    for i = 1:size(path.points,1)-1
        line_coords_x = [line_coords_x path.points(i,1) path.points(i+1,1), NaN];
        line_coords_y = [line_coords_y path.points(i,2) path.points(i+1,2), NaN];
    end
    h3.XData = line_coords_x;
    h3.YData = line_coords_y;
    drawnow
end
    save('pose_obstacle')