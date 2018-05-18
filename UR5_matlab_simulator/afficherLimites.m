function afficherLimites(limit)
%afficheur de limites
n=length(limit.limite);
cmap = hsv(n);
for i=1:n
    if limit.limite(i).opp ==0
        line_opp=0;
    else
        line_opp=1;
    end
    if all(limit.limite(i).type=='poly')
        pt_x=[limit.limite(i).surfaces.surface1(:,1);limit.limite(i).surfaces.surface2(:,1)];
        pt_y=[limit.limite(i).surfaces.surface1(:,2);limit.limite(i).surfaces.surface2(:,2)];
        pt_z=[limit.limite(i).surfaces.surface1(:,3);limit.limite(i).surfaces.surface2(:,3)];
        T = delaunay(pt_x,pt_y,pt_z);
        X=[pt_x,pt_y,pt_z];
        tetramesh(T,X,'faceColor',cmap(i,:),'EdgeColor',[0 0 0],'FaceAlpha',limit.limite(i).opp,'EdgeAlpha',line_opp);
        hold on
        view(3)
    elseif all(limit.limite(i).type=='tube')
        cyl =[limit.limite(i).surfaces.base limit.limite(i).surfaces.base+limit.limite(i).surfaces.longueur*limit.limite(i).surfaces.axe limit.limite(i).surfaces.dia/2];
        [v f] = cylinderMesh(cyl);
        drawMesh(v, f, 'faceColor',cmap(i,:),'FaceAlpha',limit.limite(i).opp,'EdgeAlpha',line_opp);
        view(3); axis equal;
        hold on
    elseif all(limit.limite(i).type=='sphe')
        s=[limit.limite(i).centroide,limit.limite(i).radius];
        [v f] = sphereMesh(s);
        drawMesh(v, f, 'faceColor',cmap(i,:),'FaceAlpha',limit.limite(i).opp,'EdgeAlpha',line_opp);
        hold on
    end
end