function [ k ] = gain_force( gain, dist_from_obj, d0,d_start )

if dist_from_obj<(d0+d_start)
    k=gain*(d_start+d0-dist_from_obj);
else
    k=0;
end


end

