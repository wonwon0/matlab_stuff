function [v_output]=verifVitesse_v9(v_input,vec_norm,d_min,thresh)

        Axe=v_input';        % command by the user
        
        
        if size(vec_norm,1)>0
            Limitation = vec_norm';
        else
             Limitation = vec_norm';
        end
        Axe;
        Limitation = Limitation(:,abs(sum(Limitation))>0.001);    % filter 0 vectors
        
        % find active translation limitations
        [LimitationActive, d_active] = find_active_limitations(Axe,Limitation, d_min, thresh);
        

        % repect translation limitations
        AxeRes = limit_manager_v3(LimitationActive,Axe);
        
        
        % small proportionnal gain controller for distance to limitation
%         [inv_command_trans] = limitation_distance_controller(AxeRes_trans, LimitationActive_trans, d_active_trans, thresh);
%         [inv_command_rot] = limitation_distance_controller(AxeRes_rot, LimitationActive_rot, d_active_rot, thresh);
% 
%         
%         v_output=(AxeRes + [inv_command_trans; inv_command_rot])';
        
        v_output = AxeRes';
end