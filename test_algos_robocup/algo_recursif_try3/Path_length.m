function [ len ] = Path_length( path )
    len = 0;
    
    for i = 1:size(path.points,1)-1
        len = len + norm(path.points(i,:)-path.points(i+1,:));
    end


end

