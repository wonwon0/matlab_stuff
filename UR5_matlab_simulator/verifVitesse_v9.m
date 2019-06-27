function [v_output]=verifVitesse_v9(v_input,vec_norm,d_min,min_distance)

        Axe=v_input';        % command by the user
        inv_command = 0;
        
        if size(vec_norm,1)>0
            Limitation = vec_norm';
        else
             Limitation = vec_norm';
        end
        Limitation(abs(Limitation)<0.000000001) = 0;
        %Limitation = Limitation(:,abs(sum(Limitation))>0.001);    % filter 0 vectors
        
        % find active limitations
        [LimitationActive, d_active] = find_active_limitations(Axe,Limitation, d_min);
        

        % repect translation limitations
        AxeRes = limit_manager_v3(LimitationActive,Axe);
        
        
        % small proportionnal gain controller for distance to limitation
        [inv_command] = limitation_distance_controller(AxeRes, LimitationActive, d_active, min_distance);
        
        v_output=(AxeRes + inv_command*0.1)';
        
        %v_output = AxeRes';
end