function [LimitationActive, d_active] = find_active_limitations(v_input,limitations, d_min, thresh)

    LimitationActive = [];      % Active limits detected
    d_active = [];        % Distance of active limits
    
    for i=1:size(limitations,2)
        limitations(:,i)=limitations(:,i)/norm(limitations(:,i));
       if ((v_input'*limitations(:,i)) <= 0) && d_min(i)<thresh && d_min(i)>0
           LimitationActive=[LimitationActive,limitations(:,i)/norm(limitations(:,i))];
           d_active = [d_active, d_min(i)];
       end
    end
end

