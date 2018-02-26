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
        Limitation;
        % Find the active limitations
        for i=1:size(Limitation,2) 
            Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
           if ((Axe'*Limitation(:,i)) <= -0.000001)
               numberLimitActive=numberLimitActive+1;
               LimitationActive=[LimitationActive,Limitation(:,i)/norm(Limitation(:,i))];
               d_active = [d_active, d_min(i) / thresh];
%                    ratio_proximite=[ratio_proximite, 1-d_min(i)/limit_proximite];
           end
        end
        d_active;
        numberLimitActive;
        Limitation;
        Axe=v_input';
        AxeRes=limit_manager_v2(LimitationActive,Axe);
%         for j=1:size(Limitation,2) % Check that the resulting vector satisfies all limitations.
%             
%                 LimitationActive=[LimitationActive,Limitation(:,j)/norm(Limitation(:,j))];
%                 d_active = [d_active, d_min(j) / thresh];
%                 Axe=v_input';
%                 numberLimitActive=numberLimitActive+1;
%                 AxeRes=limit_manager_v2(LimitationActive,Axe);
%             
%         end

         mult=0;
%         if isempty(d_min)>0
%             mult=0;
%         else
%             mult= max(abs(d_min))*10;
%         end
        
        v_output=AxeRes'-mult*v_input;
        
        
end