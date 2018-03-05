function [v_output]=verifVitesse_v8_1(v_input,vec_norm,d_min,thresh)

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
        % Find the active limitations translation
        for i=1:size(Limitation,2) 
            Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
           if ((Axe(1:3)'*Limitation(1:3,i)) <= -0.000001)
               numberLimitActive=numberLimitActive+1;
               LimitationActive=[LimitationActive,Limitation(1:3,i)/norm(Limitation(1:3,i))];
               d_active = [d_active, d_min(i) / thresh];
%                    ratio_proximite=[ratio_proximite, 1-d_min(i)/limit_proximite];
           end
        end
        LimitationActive;
        Axe=v_input';
        AxeRes_trans = limit_manager(numberLimitActive, LimitationActive,Axe(1:3));
        d_active = [];
        numberLimitActive = 0;
        LimitationActive = [];
        for i=1:size(Limitation,2) 
            Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
           if 1
               if norm(Limitation(4:6,i)) == 0
                   continue
               end
               numberLimitActive=numberLimitActive+1;
               LimitationActive=[LimitationActive,Limitation(4:6,i)/norm(Limitation(4:6,i))];
               d_active = [d_active, d_min(i) / thresh];
%                    ratio_proximite=[ratio_proximite, 1-d_min(i)/limit_proximite];
           end
        end
        LimitationActive
        AxeRes_rot = limit_manager(numberLimitActive, LimitationActive,Axe(4:6));
        AxeRes = [AxeRes_trans; AxeRes_rot];
         mult=0;
%         if isempty(d_min)>0
%             mult=0;
%         else
%             mult= max(abs(d_min))*10;
%         end
        v_output=AxeRes'-mult*v_input;
        
        
end