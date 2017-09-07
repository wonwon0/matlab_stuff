function [v_output]=verifVitesseAlex(v_input, d_min, poses_prox, pose_act, L, theta)
        Axe=v_input';

        numberLimitActive = 0;  % Number of active limits detected
        LimitationActive=[];    % Active limits detected
        LimitationCross=[];     % Vector between two limitation planes
        nbCross=0;              % Number of common lines between planes (number of 2 planes combinations) Might not be used.
        nbCrossOK=0;            % Number of common lines between planes that are not equals.
        nbCrossOK2=0;           % Number of accepted vectors
        VectbGood=1;            % If resulting vector accepted
        Vectb=[];               % Resulting vector
        Vectbkeep=[];           % Final vector
        AxeRes=[0 0 0]';
        vect_ok=[];
        vectd_ok=[];
        vect_nok=[];
        d_min_ok=[];
        d_min_nok=[];
        ratio_proximite=[];
        limit_proximite=0.01;
        nb_poses=size(pose_act,1);
        %for p=1:length(poses_prox.pose)
        for p=1:1
            pose_prox=poses_prox.pose(p).poses;
            n=length(pose_prox);
            if p==1 %correspond à l'effecteur
                for i=1:n
                    %for j=1:size(pose_act,1)
                    for j=1:1
                        vec_norm(i,:)=(pose_act(j,:)-pose_prox(i,:))/norm(pose_act(j,:)-pose_prox(i,:));
                    end
                end
            else %on examine un point qui n'est pas l'effecteur
                for i=1:n
                    for j=1:size(pose_act,1)
                        normale_pts(j,:)=(pose_act(j,:)-pose_prox(i,:))/norm(pose_act(j,:)-pose_prox(i,:));
                    end
                    normale_effecteur=PointToEffector(normale_pts, theta, L(1,:), L(p,:));
                    vec_norm=[vec_norm; normale_effecteur];
                end
            end
        end
        Limitation=vec_norm';
        % Find the active limitations
            for i=1:size(Limitation,2) 
                Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
                Limitation;
               if ((Axe'*Limitation(:,i)) < 0) && (d_min(i)<limit_proximite)
                   numberLimitActive=numberLimitActive+1;
                   LimitationActive=[LimitationActive,Limitation(:,i)/norm(Limitation(:,i))];
%                    ratio_proximite=[ratio_proximite, 1-d_min(i)/limit_proximite];
                   ratio_proximite=1;
               end
            end
        numberLimitActive;
        if numberLimitActive==0 % If no active limitations  
            AxeRes=Axe; % The resulting vector is the initial vector
        elseif numberLimitActive==1 % If 1 active limitation
            Vecta=(Axe'*LimitationActive)*LimitationActive; % Component along restriceted direction
            AxeRes=Axe-ratio_proximite*Vecta; % Remove the component from the vector
        else %If several limit active
            for i=1:numberLimitActive % For all combination pairs between limitations planes
                for j=i+1:numberLimitActive
                    nbCross=nbCross+1;%increment the number of intersections
                    LimitationCross = cross(LimitationActive(:,i),LimitationActive(:,j));%Vector perpendicular to both planes
                    if norm(LimitationCross)>0.0001 %If the planes are not parallel
                        nbCrossOK=nbCrossOK+1;%Increment the number of non-parallel combinations
                        LimitationCross=LimitationCross/norm(LimitationCross);%Normalize
                        Vectb(:,nbCrossOK)=(Axe'*LimitationCross)*LimitationCross;%Component of the vector along the common vector
                    
                        % Vector in plane
                        Vectc1=cross(LimitationActive(:,i),LimitationCross); % Vector in plane 1, perpendicular to the common vector.
                        Vectc1=Vectc1/norm(Vectc1);
                        
                        Vectc2=cross(LimitationActive(:,j),LimitationCross); % Vector in plane 2, perpendicular to the common vector.
                        Vectc2=Vectc2/norm(Vectc2);

                        Vectd1=(Axe'*Vectc1)*Vectc1; % Component in plane 1 along VectC1
                        Vectd2=(Axe'*Vectc2)*Vectc2; % Component in plane 2 along VectC2
                        Vectd1good=1;
                        Vectd2good=1;
                        if Vectd1'*LimitationActive(:,j)<0  % Verify if Vectd1 goes in plane 2
                            Vectd1good=0;
                        end
                        if Vectd2'*LimitationActive(:,i)<0  % Verify if Vectd2 goes in plane 1  
                            Vectd2good=0;
                        end
                        
                        if Vectd1good == 1 % Only one of the two vector should be good. 
                            Vectd(:,nbCrossOK)=Vectd1;
                        elseif Vectd2good==1
                            Vectd(:,nbCrossOK)=Vectd2;
                        end
                    end
                end
            end
            for i=1:size(Vectb,2)
                for j=1:size(LimitationActive,2)
                    if Vectb(:,i)'*LimitationActive(:,j)<=0
                        vect_ok=[vect_ok, Vectb(:,i)];
                        d_min_ok=[d_min_ok, d_min(j)];
                    else
                        vect_nok=[vect_nok, Vectb(:,i)];
                        d_min_nok=[d_min_nok, d_min(j)];
                    end
                end
            end
            vect_ok
            if exist('Vectd','var')
                for i=1:size(Vectd,2)
                    for j=1:size(LimitationActive,2)
                        if Vectd'*LimitationActive(:,j)<=0
                            vectd_ok=[vectd_ok, Vectd(:,i)];
                        end
                    end
                end
            end
            for i=1:size(vect_ok,2)
                AxeRes=AxeRes+vect_ok(:,i);
            end
            for i=1:size(vectd_ok,2)
                AxeRes=AxeRes+vectd_ok(:,i);
            end
%             for i=1:size(vect_nok,2)
%                 d_min_nok(i)/limit_proximite
%                 AxeRes=AxeRes+(d_min_nok(i)/limit_proximite)*vect_nok(:,i);
%             end
            if AxeRes==([0 0 0]');
                ;
%             elseif isempty(vect_ok)||size(vect_ok,2)>6
%                 AxeRes=([0 0 0]');
            else
                if norm(AxeRes)>1
                    AxeRes=AxeRes/norm(AxeRes);
                else
                    AxeRes=AxeRes;
                end
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
%         for j=1:size(Limitation,2) % Check that the resulting vector satisfies all limitations.
%             if (AxeRes'*Limitation(:,j))<-0.000000000001
%                 AxeRes = [0;0;0];
%             end
%         end
        
        v_output=AxeRes';
        
        
end