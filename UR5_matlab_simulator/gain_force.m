function [ k ] = gain_force( gain, d_min, d0,d_start )

if d_min<(d0+d_start)
    k=gain*(d_start+d0-d_min);
else
    k=0;
end


end

