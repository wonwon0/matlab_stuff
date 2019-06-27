function [inv_command] = limitation_distance_controller(v_input, LimitationActive, d_active, thresh)
    inv_command = zeros(size(v_input,1),1);
    [min_d_active, index_d_active] = min(d_active);
    if min_d_active <(thresh-0.001)
        mult = 1 - min_d_active/thresh;
        inv_command = (LimitationActive(:,index_d_active).*((v_input.*LimitationActive(:,index_d_active))<0)) * mult;
    end
    
end

