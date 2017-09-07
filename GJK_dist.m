function [dist,pts ] = GJK_dist( shape1, shape2 )

% d=rand(1,3);
d=[1 1 1];
%pick a point
a=support(shape1,shape2,d);
%pick a second point to creat a line
b=support(shape1,shape2,-d);
%pick a third point to make a tirangle
ab=b-a;
ao=-a;
%perpendicular direction
d=cross(cross(ab,ao),ab);
c=support(shape1,shape2,d);
pts=[a b c];
itt=0;
pts_old=pts;
d_old=d;
while true
    if isequal(c,b) || isequal(c,a) %we have already converged (!) and the closest point lies on the line bc or ac
        d=d+[0 0 1];
        c=support(shape1,shape2,d);
    else
        break
    end
end
while true
    %we compute the norm of vector going from the origin to each member
    %of the simplex
    Norm_pts = (sqrt(sum(abs(pts').^2,2)));
    %Computing and removing the farthest point of the simplex from the origin
    [max_norm,index]=max(Norm_pts);

    %new search direction
    if isequal(pts(:,3),pts(:,2)) || isequal(pts(:,3),pts(:,1))
        d=cross(cross(pts(:,2)-pts(:,1),-pts(:,1)),pts(:,2)-pts(:,1));
    elseif isequal(pts(:,1),pts(:,2))
        d=cross(cross(pts(:,3)-pts(:,1),-pts(:,1)),pts(:,3)-pts(:,1));
    else
        
        for i=1:3
            scal(i)=pts(:,1)'*pts(:,i);
        end
        if abs(sum(sign(scal)))==3
            list=[1 2 3];
            list(index)=[];
            d=cross(cross(pts(:,list(2))-pts(:,list(1)),-pts(:,list(1))),pts(:,list(2))-pts(:,list(1)));
        else
            d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
            if d'*pts(:,1)>0
                d=-d;
            end
        end
            
    end
    %adding a new point to the simplex.
    c=support(shape1,shape2,d)
%     if norm(c)>=max_norm || any(abs(Norm_pts-norm(c))<0.000001)
    if  any(abs(Norm_pts-norm(c))<0.000001)||all(abs(d)-abs(d_old)<0.01)
        break;
    end
    pts(:,index)=[];
    pts=[pts c]
    if norm(pts)==norm(pts_old)
        break
    end
    d_old=d;
    pts_old=pts;
    itt=itt+1;
%         pause(0.1)
end
    if isequal(pts(:,3),pts(:,2)) || isequal(pts(:,3),pts(:,1)) || isequal(pts(:,1),pts(:,2)) %we have already converged (!) and the closest point lies on the line bc or ac
%         d4 = distancePointLine3d([0 0 0], [pts(:,3)',(-pts(:,2)+pts(:,3))']);
%         d5 = distancePointLine3d([0 0 0], [pts(:,3)',(-pts(:,1)+pts(:,3))']);
%         d6 = distancePointLine3d([0 0 0], [pts(:,2)',(-pts(:,1)+pts(:,2))']);

        d1=norm(pts(:,1));
        d2=norm(pts(:,2));
        d3=norm(pts(:,3));
        d4 = distLinSeg(pts(:,1)',pts(:,2)',[0 0 0],[0 0 0]);
        d5 = distLinSeg(pts(:,1)',pts(:,3)',[0 0 0],[0 0 0]);
        d6 = distLinSeg(pts(:,2)',pts(:,3)',[0 0 0],[0 0 0]);
        dist=min([d1 d2 d3 d4 d5 d6]);
        dist=[0 0 0 0 0 0];
    else
        
        dist = pointTriangleDistance(pts',[0 0 0]);
%         dist=min([d1 d2 d3 d4]);
    end

end




















