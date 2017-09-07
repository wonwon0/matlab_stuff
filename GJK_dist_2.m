function [dist,pts ] = GJK_dist_2( shape1, shape2 )

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

while true
    a_old=a;
    b_old=b;
    c_old=c;
    ab = b-a;
    ao = -a;
    ac = c-a;
    v = cross(cross(ab,ao),ab);
    %Normal to face of triangle
    abc = cross(ab,ac);

    %Perpendicular to AB going away from triangle
    abp = cross(ab,abc);
    %Perpendicular to AC going away from triangle
    acp = cross(abc,ac);

    if dot(abp,ao) > 0
        c = b; %Throw away the furthest point and grab a new one in the right direction
        b = a;
        v = abp; %cross(cross(ab,ao),ab);

    %Is origin above (outside) AC?
    elseif dot(acp, ao) > 0
        b = a;
        v = acp; %cross(cross(ac,ao),ac);

    else
%         pts=[a b c];
%         break
    end
    a = support(shape2,shape1,v);
    [a b c]-[a_old b_old c_old]
    if isequaln([a b c],[a_old b_old c_old])
        pts=[a b c];
        break;
    end
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
    else
        
        dist = pointTriangleDistance(pts',[0 0 0]);
%         dist=min([d1 d2 d3 d4]);
    end


% 
end




















