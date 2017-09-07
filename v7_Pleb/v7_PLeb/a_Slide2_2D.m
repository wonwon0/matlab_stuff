function [AxeRes,LimitationActive]=a_Slide2_2D(Limitation, Axe)

        numberLimitActive = 0;  % Number of active limits detected
        LimitationActive=[];    % Active limits detected
        nbCrossOK=0;            % Number of common lines between planes that are not equals.
        nbCrossOK2=0;           % Number of accepted vectors
        VectbGood=1;            % If resulting vector accepted
        Vectb=[];               % Resulting vector
        Vectbkeep=[];           % Final vector
        
        for i=1:size(Limitation,2) 
            if norm(Limitation(:,i))>=0
               Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
               if (Axe'*Limitation(:,i)) < 0
                   numberLimitActive=numberLimitActive+1;
                   LimitationActive=[LimitationActive,Limitation(:,i)/norm(Limitation(:,i))];
               end
            end
        end
        
        if numberLimitActive==0 % If no active limitations  
            AxeRes=Axe; % The resulting vector is the initial vector
        elseif numberLimitActive==1 % If 1 active limitation
            Vecta=(Axe'*LimitationActive)*LimitationActive; % Component along restriceted direction
            AxeRes=Axe-Vecta; % Remove the component from the vector
        else %If several limit active
            for i=1:numberLimitActive % For all combination pairs between limitations planes
                Vecta=(Axe'*LimitationActive(:,i))*LimitationActive(:,i); % Component along restriceted direction
                Vectb(:,i)=Axe-Vecta; % Remove the component from the vector
            end
            
            for i=1:size(Vectb,2) % For each resulting vector
                VectbGood=1;% initialize at 1
                for j=1:size(Limitation,2)% check for all limitations
                    if (Vectb(:,i)'*Limitation(:,j))<-0.000000000001 %if the resulting vector goes in the limitation
                        VectbGood = 0;
                    end
                end
                if VectbGood==1 % If the vector is good
                    Vectbkeep=Vectb(:,i);%Keep the vector as the final vector
                    nbCrossOK2=nbCrossOK2+1;%Increase the number of accepted vector.
                end % we assume here that only one is good since we overwrite in Vectbkeep. nbCrossOK2 can be equal to 2 or more but each vector should be the same. This should be verified more thoroughly.
            end
            if nbCrossOK2>0 %If one vector is accepted.
                AxeRes = Vectbkeep;%we keep the vector
            else
                AxeRes=[0;0;0];%else set to zero
            end
            
        end
        for j=1:size(Limitation,2) % Check that the resulting vector satisfies all limitations.
            if (AxeRes'*Limitation(:,j))<-0.000000000001
                AxeRes = [0;0;0];
            end
        end
        
end