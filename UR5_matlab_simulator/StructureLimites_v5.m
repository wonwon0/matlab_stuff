function limit=StructureLimites_v5()
    %Fabriquer les solides ind�pendemment de leur emplacement futurs. D�inir la
    %variable "offset" comme �tant l'emplacement futur du centroide du soilde
    %que vous d�finissez.
    n = 18;
    i = 0;
    
    
    %solide -1
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface1=[0 0 0;
                                        0.4 0 0;
                                        0.4 0.05 0;
                                        0 0.05 0;
                                        0 0 0] * 1000;
    limit.limite(i).surfaces.surface2=[0 0 0.15;
                                        0.4 0 0.15;
                                        0.4 0.05 0.15;
                                        0 0.05 0.15;
                                        0 0 0.15] * 1000;
    limit.limite(i).rayonProxy=0.6 * 1000;
    limit.limite(i).offset=[-0.45 0.58 0.53] * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).collision_filter=ones(1,n);
    
    
    
    %solide 0
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface1=[0 0 0;
                                        0.35 0 0;
                                        0.35 0.28 0;
                                        0 0.28 0;
                                        0 0 0] * 1000;
    limit.limite(i).surfaces.surface2=[0 0 0.48;
                                        0.35 0 0.48;
                                        0.35 0.28 0.48;
                                        0 0.28 0.48;
                                        0 0 0.48] * 1000;
    limit.limite(i).rayonProxy=0.6 * 1000;
    limit.limite(i).offset=[-0.5 -0.4 0.16] * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).collision_filter=ones(1,n);
    
    %solide 1
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface1=[0 0 0;
                                        0.15 0 0;
                                        0.15 0.5 0;
                                        0 0.5 0;
                                        0 0 0] * 1000;
    limit.limite(i).surfaces.surface2=[0 0 0.05;
                                        0.15 0 0.05;
                                        0.15 0.5 0.05;
                                        0 0.5 0.05;
                                        0 0 0.05] * 1000;
    limit.limite(i).rayonProxy=0.6 * 1000;
    limit.limite(i).offset=[-0.4 0 0.90] * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).collision_filter=ones(1,n);

    %solide 2
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface2=[0 0 0.3;
                                        3 0 0.3;
                                        3 3 0.3;
                                        0 3 0.3;
                                        0 0 0.3] * 1000;
    limit.limite(i).surfaces.surface1=[0 0 0;
                                        3 0 0;
                                        3 3 0;
                                        0 3 0;
                                        0 0 0] * 1000;
    limit.limite(i).rayonProxy=1 * 1000;
    limit.limite(i).offset=[0 0 -0.2] * 1000;
    limit.limite(i).opp=1;
    limit.limite(i).collision_filter=ones(1,n);
    
    % %solide 3
    i = i + 1;
    limit.limite(i).type='poly';
    limit.limite(i).surfaces.surface1=[-0.08 -0.47 -0.05;
                                        0.04 -0.47 -0.05;
                                        0.04 -0.16 -0.05;
                                        -0.08 -0.16 -0.05;
                                        -0.08 -0.47 -0.05] * 1000;
    limit.limite(i).surfaces.surface2=[-0.08 -0.47 0.15;
                                        0.04 -0.47 0.15;
                                        0.04 -0.16 0.15;
                                        -0.08 -0.16 0.15;
                                        -0.08 -0.47 0.15] * 1000;
    limit.limite(i).rayonProxy=sqrt(3*0.1^2)+0.4 * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[-0.3 -0.2 0] * 1000;
    limit.limite(i).collision_filter=ones(1,n);
    % %solide 4
    i = i + 1;
    limit.limite(i).type='poly';
    limit.limite(i).surfaces.surface1=[0 -0.47 -0.05;
                                        0.4 -0.47 -0.05;
                                        0.4 -0.16 -0.05;
                                        0 -0.16 -0.05;
                                        0 -0.47 -0.05] * 1000;
    limit.limite(i).surfaces.surface2=[0 -0.47 0.08;
                                        0.4 -0.47 0.08;
                                        0.4 -0.16 0.08;
                                        0 -0.16 0.08;
                                        0 -0.47 0.08] * 1000;
    limit.limite(i).rayonProxy=sqrt(3*0.1^2)+0.4 * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[-0.4 0.4 0.4] * 1000;
    limit.limite(i).collision_filter=ones(1,n);
    % %solide 5
    i = i + 1;
    limit.limite(i).type='tube';
    limit.limite(i).surfaces.axe=[0 0 1]/norm([0 0 1]);
    limit.limite(i).surfaces.base=[0 0 0];
    limit.limite(i).surfaces.longueur=0.5 * 3000;
    limit.limite(i).surfaces.dia=0.25 * 1000;
    limit.limite(i).rayonProxy=limit.limite(i).surfaces.longueur;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[0 0 -0.05] * 1000;
    limit.limite(i).collision_filter=zeros(1,n);
    limit.limite(i).collision_filter(1:4) = 1;


%     solide 6
    i = i + 1;
    limit.limite(i).type='sphe';
    limit.limite(i).radius=0.85 * 1000;
    limit.limite(i).rayonProxy=2 * 1000;
    limit.limite(i).centroide=[0 0 0] * 1000;
    limit.limite(i).opp=0.1;
    limit.limite(i).collision_filter=zeros(1,n);
    limit.limite(i).collision_filter(4) = 1;
    
    
    % %solide 7
    i = i + 1;
    limit.limite(i).type='poly';
    limit.limite(i).surfaces.surface1=[0 -0.47 -0.05;
                                        0.1 -0.47 -0.05;
                                        0.1 -0.16 -0.05;
                                        0 -0.16 -0.05;
                                        0 -0.47 -0.05] * 1000;
    limit.limite(i).surfaces.surface2=[0 -0.47 0.18;
                                        0.1 -0.47 0.18;
                                        0.1 -0.16 0.18;
                                        0 -0.16 0.18;
                                        0 -0.47 0.18] * 1000;
    limit.limite(i).rayonProxy=sqrt(3*0.1^2)+0.4 * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[-0.2 0.4 0.5] * 1000;
    limit.limite(i).collision_filter=ones(1,n);
    % solide 8
    i = i + 1;
    limit.limite(i).type='poly';
    limit.limite(i).surfaces.surface1=[0 -0.37 0.05;
                                        0.4 -0.37 0.05;
                                        0.4 -0.16 0.05;
                                        0 -0.16 0.05;
                                        0 -0.37 0.05] * 1000;
    limit.limite(i).surfaces.surface2=[0 -0.37 0.08;
                                        0.4 -0.37 0.08;
                                        0.4 -0.16 0.08;
                                        0 -0.16 0.08;
                                        0 -0.37 0.08] * 1000;
    limit.limite(i).rayonProxy=sqrt(3*0.1^2)+0.4 * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[-0.4 0.45 0.6] * 1000;
    limit.limite(i).collision_filter=ones(1,n);
    
    
    % solide 9
    i = i + 1;
    limit.limite(i).type='poly';
    limit.limite(i).surfaces.surface1=[0 -0.37 0.05;
                                        0.55 -0.37 0.05;
                                        0.55 -0.16 0.05;
                                        0 -0.16 0.05;
                                        0 -0.37 0.05] * 1000;
    limit.limite(i).surfaces.surface2=[0 -0.37 0.08;
                                        0.55 -0.37 0.08;
                                        0.55 -0.16 0.08;
                                        0 -0.16 0.08;
                                        0 -0.37 0.08] * 1000;
    limit.limite(i).rayonProxy=sqrt(3*0.1^2)+0.4 * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).offset=[-0.55 -0.2 0.15] * 1000;
    limit.limite(i).collision_filter=ones(1,n);

    % solide 10
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface1=[0 0 0;
                                        0.025 0 0;
                                        0.025 0.5 0;
                                        0 0.5 0;
                                        0 0 0] * 1000;
    limit.limite(i).surfaces.surface2=[0 0 0.2;
                                        0.025 0 0.2;
                                        0.025 0.5 0.2;
                                        0 0.5 0.2;
                                        0 0 0.2] * 1000;
    limit.limite(i).rayonProxy=0.6 * 1000;
    limit.limite(i).offset=[-0.450 0 0.8] * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).collision_filter=ones(1,n);
    
    % solide 11
    i = i + 1;
    limit.limite(i).type='poly';

    limit.limite(i).surfaces.surface1=[0 0 0;
                                        0.025 0 0;
                                        0.025 0.5 0;
                                        0 0.5 0;
                                        0 0 0] * 1000;
    limit.limite(i).surfaces.surface2=[0 0 0.2;
                                        0.025 0 0.2;
                                        0.025 0.5 0.2;
                                        0 0.5 0.2;
                                        0 0 0.2] * 1000;
    limit.limite(i).rayonProxy=0.6 * 1000;
    limit.limite(i).offset=[-0.300 0 0.8] * 1000;
    limit.limite(i).opp=0.3;
    limit.limite(i).collision_filter=ones(1,n);

    %Definition des centroides de chaque solides
    for i=1:length(limit.limite)
        if strcmp(limit.limite(i).type,'tube')
            limit.limite(i).centroide=limit.limite(i).surfaces.axe*limit.limite(i).surfaces.longueur/2+limit.limite(i).surfaces.base;
        elseif strcmp(limit.limite(i).type, 'poly')
            limit.limite(i).centroide=[(sum(limit.limite(i).surfaces.surface1(:,1))+sum(limit.limite(i).surfaces.surface2(:,1)))/(length(limit.limite(i).surfaces.surface1(:,1))*2) 
                                        (sum(limit.limite(i).surfaces.surface1(:,2))+sum(limit.limite(i).surfaces.surface2(:,2)))/(length(limit.limite(i).surfaces.surface1(:,2))*2)
                                       (sum(limit.limite(i).surfaces.surface1(:,3))+sum(limit.limite(i).surfaces.surface2(:,3)))/(length(limit.limite(i).surfaces.surface1(:,3))*2)]';
        end
    end
    % limit.limite(3).offset=limit.limite(3).centroide;
    %division des solides s'ils n'ont pas de sections constantes

    % limit.limite(6).offset=limit.limite(6).centroide;
    dup=[];
    dz=0.06;

    for i=1:length(limit.limite)
        if strcmp(limit.limite(i).type, 'poly')
            s=size(limit.limite(i).surfaces.surface1);
            if isequal(limit.limite(i).surfaces.surface1(:,1:2),limit.limite(i).surfaces.surface2(:,1:2))==0
                limit.limite(i).surfaces.surface1(:,1:2)
                limit.limite(i).surfaces.surface2(:,1:2)
                n=floor(abs((limit.limite(i).surfaces.surface2(1,3)-limit.limite(i).surfaces.surface1(1,3))/dz));
                dec=zeros(s);
                dec(:,1:2)=(limit.limite(i).surfaces.surface2(:,1:2)-limit.limite(i).surfaces.surface1(:,1:2))/n;
                dup=[dup,i];
                for it=1:n
                    ind=length(limit.limite);
                    if it==1
                        limit.limite(ind+1).surfaces.surface1=limit.limite(i).surfaces.surface1;
                        for L=1:s(1)
                            limit.limite(ind+1).surfaces.surface2(L,:)=limit.limite(i).surfaces.surface1(L,:)+[0 0 dz];
                        end
                    else
                        for L=1:s(1)
                            limit.limite(ind+1).surfaces.surface1(L,:)=limit.limite(ind).surfaces.surface2(L,:)+dec(L,:);
                            limit.limite(ind+1).surfaces.surface2(L,:)=limit.limite(ind+1).surfaces.surface1(L,:)+[0 0 dz];
                        end
                    end
                    limit.limite(ind+1).offset=limit.limite(i).offset+(it-1)*[0 0 dz];
                    limit.limite(ind+1).rayonProxy=limit.limite(i).rayonProxy;
                    limit.limite(ind+1).centroide=[limit.limite(i).centroide(1:2),limit.limite(i).surfaces.surface1(1,3)+(it-1)*dz];
                end
            end
        end
    end

    %On enl�ve le solide qui a �t� discr�tis�.

    for i=1:length(dup)
        limit.limite(dup(i))=[];
    end

    %Positionnement des solides strcmp(limit.limite(i).type, 'poly')

    for i=1:length(limit.limite)
        if strcmp(limit.limite(i).type, 'poly')
            s=size(limit.limite(i).surfaces.surface1);
            for j = 1:s(1)
                limit.limite(i).surfaces.surface1(j,:)=limit.limite(i).surfaces.surface1(j,:)-limit.limite(i).centroide+limit.limite(i).offset;
                limit.limite(i).surfaces.surface2(j,:)=limit.limite(i).surfaces.surface2(j,:)-limit.limite(i).centroide+limit.limite(i).offset;
            end
            limit.limite(i).centroide=limit.limite(i).offset;
        elseif strcmp(limit.limite(i).type, 'tube')
            limit.limite(i).surfaces.base = limit.limite(i).offset;
        end
    end
    % rotation de l'environement par rapport a un angle donné
    angle = 0;
    mat_rot_z = [cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
    for i=1:length(limit.limite)
        if strcmp(limit.limite(i).type, 'poly')
            s=size(limit.limite(i).surfaces.surface1);
            for j = 1:s(1)
                limit.limite(i).surfaces.surface1(j,:)=(mat_rot_z * limit.limite(i).surfaces.surface1(j,:)')';
                limit.limite(i).surfaces.surface2(j,:)=(mat_rot_z * limit.limite(i).surfaces.surface2(j,:)')';
            end
            limit.limite(i).centroide=(mat_rot_z * limit.limite(i).centroide')';
        elseif strcmp(limit.limite(i).type, 'tube')
            limit.limite(i).surfaces.base = (mat_rot_z * limit.limite(i).surfaces.base')';
        elseif strcmp(limit.limite(i).type, 'sphe')
            limit.limite(i).centroide = (mat_rot_z * limit.limite(i).centroide')';
        end
    end
end
    




































