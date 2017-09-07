function [dist,pts ] = GJK_dist_5( shape1, shape2,dist_last )

% d=rand(1,3);
d=[1 1 1];
%pick a point
[a, point1_a, point2_a]=support_2(shape1,shape2,d);
%pick a second point to creat a line
[b, point1_b, point2_b]=support(shape1,shape2,-d);
%pick a third point to make a tirangle
ab=b-a;
ao=-a;
%perpendicular direction
d=cross(cross(ab,ao),ab);
[c, point1_c, point2_c]=support_2(shape1,shape2,d);
pts=[a b c];
pts_1_2=[point1_a point1_b point1_c;point2_a point2_b point2_c];
itt=0;
pts_old=pts;
norm_old=norm(pts_old);
d_old=d;
dist_old=1000;
% while true
%     if isequal(c,b) || isequal(c,a) %we have already converged (!) and the closest point lies on the line bc or ac
%         d=d+[0 0 1];
%         [c, point1_c, point2_c]=support_2(shape1,shape2,d);
%     else
%         break
%     end
% end
while true
    %we compute the norm of vector going from the origin to each member
    %of the simplex
    Norm_pts = (sqrt(sum(abs(pts').^2,2)));
    %Computing and removing the farthest point of the simplex from the origin
    [max_norm,index]=max(Norm_pts);

    %new search direction

        
%         for i=1:3
%             scal(i)=pts(:,1)'*pts(:,i);
%         end
%         if abs(sum(sign(scal)))==3
%             list=[1 2 3];
%             list(index)=[];
%             d=cross(cross(pts(:,list(2))-pts(:,list(1)),-pts(:,list(1))),pts(:,list(2))-pts(:,list(1)));
%         else
            d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
            origin_projected=(pts(:,1)'*d)*d;
            if d'*origin_projected>0
                d=-d;
            end
%         end
    d=d/norm(d);
    %adding a new point to the simplex.
    [c, point1_c, point2_c]=support_2(shape1,shape2,d);
%     if norm(c)>=max_norm || any(abs(Norm_pts-norm(c))<0.000001)
    origin_projected=(pts(:,1)'*d)*d;
    if any(abs(Norm_pts-norm(c))<0.000001)||itt>5
        
        d1 = distLinSeg(pts(:,1)',pts(:,2)',origin_projected',origin_projected');
        d2 = distLinSeg(pts(:,1)',pts(:,3)',origin_projected',origin_projected');
        d3 = distLinSeg(pts(:,2)',pts(:,3)',origin_projected',origin_projected');
%         d1=(pts(:,2)-pts(:,1))'/norm(pts(:,2)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
%         d2=(pts(:,3)-pts(:,1))'/norm(pts(:,3)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
%         d3=(pts(:,3)-pts(:,2))'/norm(pts(:,3)-pts(:,2))*(pts(:,2)-origin_projected)/norm((pts(:,2)-origin_projected));
        dist_seg_min=min(abs([d1 d2 d3]));
        if dist_seg_min<dist
            if d1==dist_seg_min
                %pick a third point to make a tirangle
                ab=pts(:,2)-pts(:,1);
                ao=-pts(:,1);
                %perpendicular direction
                d=cross(cross(ab,ao),ab);
                [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                index=3;
            elseif d2==dist_seg_min
                %pick a third point to make a tirangle
                ab=pts(:,3)-pts(:,1);
                ao=-pts(:,1);
                %perpendicular direction
                d=cross(cross(ab,ao),ab);
                [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                index=2;
            else
                %pick a third point to make a tirangle
                ab=pts(:,3)-pts(:,2);
                ao=-pts(:,2);
                %perpendicular direction
                d=cross(cross(ab,ao),ab);
                [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                index=1;
            end
        end
        if any(abs(Norm_pts-norm(c))<0.000001)
            Norm_pts=0
            break
        end
    end
    if  all(abs(d-d_old)<0.01)
        d_old=0
        break;
    end
    pts(:,index)=[];
    pts_1_2(:,index)=[];
    pts=[pts c];
    pts_1_2=[pts_1_2 [point1_c;point2_c]];
    [dist, PP0] = pointTriangleDistance(pts',[0 0 0]);

    if dist<dist_old
        pts_prox=pts;
        dist_prox=dist;
    end
    if itt>10
        pts=pts_prox;
        dist=dist_prox;
        break
    end

    dist_old=dist;
    d_old=d;
    pts_old=pts;
    if abs(norm_old-norm(pts))<0.01
        norm_old=norm(pts);
    end
    itt=itt+1
%         pause(0.1)
end
%     if isequal(pts(:,3),pts(:,2)) || isequal(pts(:,3),pts(:,1)) || isequal(pts(:,1),pts(:,2)) %we have already converged (!) and the closest point lies on the line bc or ac
% %         d4 = distancePointLine3d([0 0 0], [pts(:,3)',(-pts(:,2)+pts(:,3))']);
% %         d5 = distancePointLine3d([0 0 0], [pts(:,3)',(-pts(:,1)+pts(:,3))']);
% %         d6 = distancePointLine3d([0 0 0], [pts(:,2)',(-pts(:,1)+pts(:,2))']);
% 
%         d1=norm(pts(:,1));
%         d2=norm(pts(:,2));
%         d3=norm(pts(:,3));
%         d4 = distLinSeg(pts(:,1)',pts(:,2)',[0 0 0],[0 0 0]);
%         d5 = distLinSeg(pts(:,1)',pts(:,3)',[0 0 0],[0 0 0]);
%         d6 = distLinSeg(pts(:,2)',pts(:,3)',[0 0 0],[0 0 0]);
%         dist=min([d1 d2 d3 d4 d5 d6]);
%         dist=[0 0 0 0 0 0];
%     else
%         
%         dist = pointTriangleDistance(pts',[0 0 0]);
% %         dist=min([d1 d2 d3 d4]);
%     end

 T = [1	 4  3
      1  4  2
      3	 4  2
      1  2  3];
  pts=[pts';PP0];
  TR=triangulation(T,pts);
  B=barycentricToCartesian(TR,4,PP0);
  
end




















