function [a,b,c,flag] = pickTriangle(a,b,shape1,shape2,IterationAllowed)
flag = 0; %So far, we don't have a successful triangle.

%First try:
ab = b-a;
ao = -a;
v = cross(cross(ab,ao),ab); % v is perpendicular to ab pointing in the general direction of the origin.

c = b;
b = a;
a = support(shape2,shape1,v);

for i = 1:IterationAllowed %iterations to see if we can draw a good triangle.
    %Time to check if we got it:
    ab = b-a;
    ao = -a;
    ac = c-a;
    
    %Normal to face of triangle
    abc = cross(ab,ac);
    
    %Perpendicular to AB going away from triangle
    abp = cross(ab,abc);
    %Perpendicular to AC going away from triangle
    acp = cross(abc,ac);
    
    %First, make sure our triangle "contains" the origin in a 2d projection
    %sense.
    %Is origin above (outside) AB?   
    if dot(abp,ao) > 0
        c = b; %Throw away the furthest point and grab a new one in the right direction
        b = a;
        v = abp; %cross(cross(ab,ao),ab);
        
        %Is origin above (outside) AC?
    elseif dot(acp, ao) > 0
        b = a;
        v = acp; %cross(cross(ac,ao),ac);
        
    else
        flag = 1;
        break; %We got a good one.
    end
    a = support(shape2,shape1,v);
end
end