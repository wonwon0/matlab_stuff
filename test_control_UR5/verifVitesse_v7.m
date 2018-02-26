function [v_output]=verifVitesse_v7(v_input,vec_norm,d_min,thresh)

        Axe=v_input';
        numberLimitActive = 0;  % Number of active limits detected
        LimitationActive=[];    % Active limits detected
        d_active = [];
%         m=size(pose_act,1);
%         b=size(limit.limite,2);
%         for j=1:m
%             for i=1:b
%                 
%                 if d_min(i*j)<10
%                     vec_norm=[vec_norm;(pose_act(j,:)-pose_prox(i*j,:))/norm(pose_act(j,:)-pose_prox(i*j,:))];
%                     if j~=m
%                         vec_norm(end,:)=PointToEffector(vec_norm(end,:), theta_act, L(j,:),L(m,:),ddl(j));
%                     end
%                 end
%             end
%         end
        vec_norm;
        if size(vec_norm,1)>0
            Limitation=vec_norm(:,1:3)';
        else
             Limitation=vec_norm';
        end
        Limitation;
        % Find the active limitations
        for i=1:size(Limitation,2) 
            Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
           if ((Axe'*Limitation(:,i)) <= 0)
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
        AxeRes=limit_manager_v2(numberLimitActive,LimitationActive,Axe, d_active)
        for j=1:size(Limitation,2) % Check that the resulting vector satisfies all limitations.
            if (AxeRes'*Limitation(:,j))<-0.000000001
                LimitationActive=[LimitationActive,Limitation(:,j)/norm(Limitation(:,j))];
                d_active = [d_active, d_min(j) / thresh];
                Axe=v_input';
                numberLimitActive=numberLimitActive+1;
                AxeRes=limit_manager_v2(numberLimitActive,LimitationActive,Axe, d_active);
            end
        end
%         if nbCrossOK==0 %If all limitations are perpendicular, its as if there was one limitation
%             Vecta=(Axe'*LimitationActive(:,1))*LimitationActive(:,1);% Component along restriceted direction
%             AxeRes=Axe-ratio_proximite*Vecta;% Remove the component from the vector
%         else
%             for i=1:size(Vectb,2) % For each resulting vector
%                 VectbGood=1;% initialize at 1
%                 for j=1:size(Limitation,2)% check for all limitations
%                     if (Vectb(:,i)'*Limitation(:,j))<-0.000000000001 %if the resulting vector goes in the limitation
%                         VectbGood = 0;
%                     end
%                 end
%                 if VectbGood==1 % If the vector is good
%                     Vectbkeep=Vectb(:,i);%Keep the vector as the final vector
%                     nbCrossOK2=nbCrossOK2+1;%Increase the number of accepted vector.
%                 end % we assume here that only one is good since we overwrite in Vectbkeep. nbCrossOK2 can be equal to 2 or more but each vector should be the same. This should be verified more thoroughly.
%             end
%              if nbCrossOK2>0 %If one vector is accepted.
%                 AxeRes = Vectbkeep;%we keep the vector
%             else
%                 AxeRes=[0;0;0];%else set to zero
%             end
%         end
%         if isempty(d_min)>0
%             mult=0;
%         else
%             mult= max(abs(d_min))*10;
%         end
        mult=0;
        v_output=AxeRes'-mult*v_input;
        
        
end