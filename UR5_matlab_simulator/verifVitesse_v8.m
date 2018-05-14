function [v_output]=verifVitesse_v8(v_input,vec_norm,d_min,thresh)

        Axe=v_input';
        numberLimitActive = 0;  % Number of active limits detected
        LimitationActive=[];    % Active limits detected
        d_active = [];
        vec_norm;
        if size(vec_norm,1)>0
            Limitation=vec_norm(:,1:6)';
        else
             Limitation=vec_norm';
        end
        Axe;
        Limitation;
        % Find the active limitations
        for i=1:size(Limitation,2)
            Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
           if ((Axe'*Limitation(:,i)) <= 0) && d_min(i)<thresh
               numberLimitActive=numberLimitActive+1;
               LimitationActive=[LimitationActive,Limitation(:,i)/norm(Limitation(:,i))];
               d_active = [d_active, d_min(i)];
%                    ratio_proximite=[ratio_proximite, 1-d_min(i)/limit_proximite];
           end
        end
        d_active;
        numberLimitActive;
        LimitationActive;
        Axe=v_input';
        AxeRes=limit_manager_v3(LimitationActive,Axe);
        inv_command = zeros(6,1);
        mult=0;
        [min_d_active, index_d_active] = min(d_active);
        if min_d_active <(thresh-0.001)
            mult = 1 - min_d_active/thresh;
            inv_command = LimitationActive(:,index_d_active) * mult / 10;
        end
        v_output=(AxeRes + inv_command)';
        
        
end