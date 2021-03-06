function [v_output]=verifVitesse_trans_rot_v0(v_input,vec_norm,d_min)

        Axe=v_input';
        Axe_trans=v_input(1:3)';        % translation command by the user
        Axe_rot=v_input(4:6)';          % rotation command by the user
        if isempty(vec_norm)
            v_output = v_input;
            return
        end
        if size(vec_norm,1)>0
            Limitation_trans = vec_norm(:,1:3)';
            Limitation_rot = vec_norm(:,4:6)';
        else
             Limitation_trans = vec_norm(1:3)';
             Limitation_rot = vec_norm(4:6)';
        end
        Axe;
        %Limitation_trans = Limitation_trans(:,abs(sum(Limitation_trans))>0.001);    % filter 0 vectors
        %Limitation_rot = Limitation_rot(:,abs(sum(Limitation_rot))>0.00000001);     % filter 0 vectors
        
        % find active translation limitations
        [LimitationActive_trans, d_active_trans] = find_active_limitations(Axe_trans,Limitation_trans, d_min);
        
        % find active rotation limitations
        [LimitationActive_rot, d_active_rot] = find_active_limitations(Axe_rot, Limitation_rot, d_min);

        % repect translation limitations
        AxeRes_trans = limit_manager(size(LimitationActive_trans,2), LimitationActive_trans,Axe_trans);
        
        % repect rotation limitations
        Limitation_rot;
        LimitationActive_rot;
        Axe_rot;
        AxeRes_rot = limit_manager(size(LimitationActive_rot,2), LimitationActive_rot,Axe_rot);
        
        AxeRes = [AxeRes_trans; AxeRes_rot];
        
        % small proportionnal gain controller for distance to limitation
        [inv_command_trans] = limitation_distance_controller(AxeRes_trans, LimitationActive_trans, d_active_trans, 50);
        [inv_command_rot] = limitation_distance_controller(AxeRes_rot, LimitationActive_rot, d_active_rot, 50);

        
        v_output=(AxeRes + [inv_command_trans; inv_command_rot]*1)';
        
        v_output = AxeRes';
end