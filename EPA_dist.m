function [dist ] = GJK_dist( Shape1, Shape2 )

d=[1 1 1];
%pick a point
a=support(shape1,shape2,d);
%pick a second point to creat a line
b=support(shape1,shape2,-a);
%pick a third point to make a tirangle
ab=b-a;
ao=-a;
%perpendicular direction
d=cross(cross(ab,ao),ab);
c=support(shape1,shape2,d);
pts=[a b c];
if c==b || c==a %we have already converged (!) and the closest point is c
	dist=norm(c);
else
    while true
        %we compute the norm of vector going from the origin to each member
        %of the simplex
        Norm_pts = (sqrt(sum(abs(pts').^2,2)));
        %Computing and removing the farthest point of the simplex from the origin
        [~,index]=max(Norm_pts);
        pts_copy=pts;
        pts(:,index)=[];
        %new search direction
        d=cross(cross(pts(:,2)-pts(:,1),-pts(:,1)),pts(:,2)-pts(:,1));
        %adding a new point to the simplex.
        c=support(shape1,shape2,d);
        pts=[pts c];
        if isequal(pts,pts_copy)
            break;
        end
    end
    dist = pointTriangleDistance(pts,[0 0 0]');



end




















