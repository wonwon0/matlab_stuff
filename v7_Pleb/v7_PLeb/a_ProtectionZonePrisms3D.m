function [Limitation, inside, m] = a_ProtectionZonePrisms3D(posActu, Prisms, gap, Limitation)

inside = 0;
m=0;

    if size(Prisms,1)>0 % Si un prisme est defini
      for i=1:size(Prisms,3) % Pour tout les prismes
            inZone = 1; % On assume quon est dans le prisme
            D=[];
            for j=1:size(Prisms,1)% Pour toutes les faces
                n = Prisms(j,4:6,i)';%Normale de la face
                n=n/norm(n);%Normaliser la normale
                w = posActu - (Prisms(j,1:3,i)'+gap*n);%Distance 3D entre la pos actu et la face
                d = (n'*w); % Distance entre la pos et la face dans la direction de la normale
                D(j,:)=[abs(n'*w),n'];%Distance en absolu et normale
                if d > 0 
                    inZone = 0; % Si le point teste nest pas a linterieur dune face le point nest pas dans le prisme.
                end
            end
            if inZone == 1% Si on est dans la zone
                [m,index] = min(D(:,1));%Le point est plus proche de quelle face
                normal = D(index,2:4)';%La normale de cette face
                Limitation = [Limitation,normal];%On ajoute la normale de cette face aux limitations
                inside=1;%On est a linterieur de la face
            end
        end
    end




end








