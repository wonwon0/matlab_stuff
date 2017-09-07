function [a,b,c,dist_origin]=EPA(a,b,c,d)

    pts=[a;b;c;d]';
    % compute closest distance from segments to origin
    sets=[[1 2 3];[1 2 4];[1 3 3];[2 3 4];[1 1 2];[1 1 3];[1 1 4];[2 2 3];[2 2 4];[3 3 4]];
    dist(1) = pointTriangleDistance(pts(:,[1 2 3])',[0 0 0]);
    dist(2) = pointTriangleDistance(pts(:,[1 2 4])',[0 0 0]);
    dist(3) = pointTriangleDistance(pts(:,[1 3 4])',[0 0 0]);
    dist(4) = pointTriangleDistance(pts(:,[2 3 4])',[0 0 0]);
    dist(5) = distLinSeg(pts(:,1)',pts(:,2)',[0 0 0],[0 0 0]);
    dist(6) = distLinSeg(pts(:,1)',pts(:,3)',[0 0 0],[0 0 0]);
    dist(7) = distLinSeg(pts(:,1)',pts(:,4)',[0 0 0],[0 0 0]);
    dist(8) = distLinSeg(pts(:,2)',pts(:,3)',[0 0 0],[0 0 0]);
    dist(9) = distLinSeg(pts(:,2)',pts(:,4)',[0 0 0],[0 0 0]);
    dist(10) = distLinSeg(pts(:,3)',pts(:,4)',[0 0 0],[0 0 0]);
        
    [dist_origin,index]=min(dist);
    pts=pts(:,sets(index,:));
    a=pts(:,1)';b=pts(:,2)';c=pts(:,3)';
%     dist(1)=cross(abcd(:,1)-abcd(:,2),abcd(:,3)-abcd(:,1))'*abcd(:,2);
%     dist(2)=cross(abcd(:,2)-abcd(:,3),abcd(:,4)-abcd(:,2))'*abcd(:,3);
%     dist(3)=cross(abcd(:,3)-abcd(:,4),abcd(:,1)-abcd(:,3))'*abcd(:,4);
%     dist(4)=cross(abcd(:,4)-abcd(:,1),abcd(:,2)-abcd(:,4))'*abcd(:,1);
%     for i=1:length(dist)
%         if dist(i)==min(dist)
%             liste=mod(i:i+2,4);
%             liste=liste+ismember(liste,0)*4;
%             points=abcd(:,liste);
%             dist_origin=cross(points(:,1)-points(:,2),points(:,1)-points(:,3))'*points(:,1);
%             a=points(:,1);b=points(:,2);c=points(:,3);
%         end
%     end
% end