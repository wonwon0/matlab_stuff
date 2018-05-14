function [h] = display_robot_UR5_v1(h, poses_articulations)

mat_ligne_x=[0,0];
mat_ligne_y=[0,0];
mat_ligne_z=[0,0];
if 1
    for p=size(poses_articulations, 2):-1:1
        mat_ligne_x=[mat_ligne_x poses_articulations(1,p)];
        mat_ligne_y=[mat_ligne_y poses_articulations(2,p)];
        mat_ligne_z=[mat_ligne_z poses_articulations(3,p)];
%                     h8(i+(p-1)*t)=line([pose_act(p,1) pose(i,1)],[pose_act(p,2) pose(i,2)],[pose_act(p,3) pose(i,3)],'LineWidth',2,'color','r');
    end
%             h8=line(mat_ligne_x,mat_ligne_y,mat_ligne_z,'LineWidth',2,'color','r');
    h.XData=mat_ligne_x;
    h.YData=mat_ligne_y;
    h.ZData=mat_ligne_z;
else
    t=9;
    mat_ligne_x=[NaN];
    mat_ligne_y=[NaN];
    mat_ligne_z=[NaN];
    h.XData=mat_ligne_x;
    h.YData=mat_ligne_y;
    h.ZData=mat_ligne_z;
end
end

