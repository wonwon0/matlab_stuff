i=322;

close all

S1.Vertices = stock.s1{i}.vert;
S1.Faces = stock.s1{i}.faces;
S1.FaceVertexCData = jet(size(V1,1));
S1.FaceColor = 'interp';
S1Obj = patch(S1);


S2.Vertices = stock.s2{i}.vert;
S2.Faces = stock.s2{i}.faces;
S2.FaceVertexCData = jet(size(V2,1));
S2.FaceColor = 'interp';
S2Obj = patch(S2);


fig=figure;
set(fig, 'renderer', 'OpenGL');
set(fig, 'Position', [-960 40 960 960])
    a=stock.s{i};
    pts=stock.pts{i};
    pt_x=a(:,1);
    pt_y=a(:,2);
    pt_z=a(:,3);
    T = delaunay(pt_x,pt_y,pt_z);
    X=[pt_x,pt_y,pt_z];
    try delete(h1)
    end
    try delete(h2)
    end
    h1=tetramesh(T,X,'faceColor','b','EdgeColor',[0 0 0],'FaceAlpha',0.1);
    hold on
    for j=1:3
        h2(j)=plot3(pts(1,j),pts(2,j),pts(3,j),'Or','LineWidth',3);
        hold on
        d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
        if d'*pts(:,1)>0
            d=-d;
        end
        line([pts(1,1) pts(1,1)+d(1)],[pts(2,1) pts(2,1)+d(2)],[pts(3,1) pts(3,1)+d(3)])
        hold on
    end
    hold off
    
    axis([min(S1.Vertices(:,1))-0.1 max(S1.Vertices(:,1))+0.1 min(S1.Vertices(:,2))-0.1 max(S1.Vertices(:,2))+0.1 min(S1.Vertices(:,3))-0.1 max(S1.Vertices(:,3))+0.1]);
    drawnow;
    pause(0.05)
    hold on
    plot3(0, 0 ,0,'Og','Linewidth',3)
    
    
    d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
            origin_projected=(pts(:,1)'*d)*d;
            if d'*origin_projected>0
                d=-d;
            end
%         end
    d=d/norm(d);
    origin_projected=(pts(:,1)'*d)*d;
    plot3(origin_projected(1),origin_projected(2),origin_projected(3),'X','LineWidth',3)
    d1 = distLinSeg(pts(:,1)',pts(:,2)',origin_projected',origin_projected')
    d2 = distLinSeg(pts(:,1)',pts(:,3)',origin_projected',origin_projected')
    d3 = distLinSeg(pts(:,2)',pts(:,3)',origin_projected',origin_projected')
    dist_seg_min=min(abs([d1 d2 d3]))
    
    
    
    if d1==dist_seg_min
        %pick a third point to make a tirangle
        ab=pts(:,2)-pts(:,1);
        ao=-pts(:,1);
        %perpendicular direction
        d=cross(cross(ab,ao),ab)
        [c, point1_c, point2_c]=support_2(S1Obj,S2Obj,d)
        index=3
    elseif d2==dist_seg_min
        %pick a third point to make a tirangle
        ab=pts(:,3)-pts(:,1);
        ao=-pts(:,1);
        %perpendicular direction
        d=cross(cross(ab,ao),ab)
        [c, point1_c, point2_c]=support_2(S1Obj,S2Obj,d)
        index=2
    else
        %pick a third point to make a tirangle
        ab=pts(:,3)-pts(:,2);
        ao=-pts(:,2);
        %perpendicular direction
        d=cross(cross(ab,ao),ab)
        [c, point1_c, point2_c]=support_2(S1Obj,S2Obj,d)
        index=1
    end

    
    
    
    

    
    