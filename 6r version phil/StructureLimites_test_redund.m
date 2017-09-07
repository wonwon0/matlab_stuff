function limit=StructureLimites_test_redund()
%Fabriquer les solides indépendemment de leur emplacement futurs. Déinir la
%variable "offset" comme étant l'emplacement futur du centroide du soilde
%que vous définissez.

%solide 1

limit.limite(1).type='poly';

% limit.limite(1).surfaces.surface1=[-2 -2 0;
%                                     -2 2 0;
%                                     2 2 0;
%                                     2 -2 0;
%                                     -2 -2 0];
% limit.limite(1).surfaces.surface2=[-2 -2 0.4;
%                                     -2 2 0.4;
%                                     2 2 0.4;
%                                     2 -2 0.4;
%                                     -2 -2 0.4];
% limit.limite(1).rayonProxy=sqrt(3*0.1^2)+0.4;
% limit.limite(1).offset=[0 0 0];
limit.limite(1).type='poly';
limit.limite(1).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(1).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(1).rayonProxy=0.6;
limit.limite(1).offset=[-0.35 -0.3 0];
limit.limite(1).opp=0.3;

% limit.limite(1).surfaces.surface1=[0.1 0.1 0.1;
%                                     0.4 0.1 0.1;
%                                     0.4 0.4 0.1;
%                                     0.1 0.4 0.1];
% limit.limite(1).surfaces.surface2=[0.15 0.15 0.4;
%                                     0.35 0.15 0.4;
%                                     0.35 0.35 0.4;
%                                     0.15 0.35 0.4];
% limit.limite(1).rayonProxy=sqrt(3*0.1^2)+0.4;
% limit.limite(1).offset=[0.3 0.3 0.1];

%solide 2
limit.limite(2).type='poly';
limit.limite(2).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(2).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(2).rayonProxy=0.6;
limit.limite(2).offset=[-0.35 -0.33 0];
limit.limite(2).opp=0.3;

% limit.limite(2).surfaces.surface1=[0.3 0.3 0;
%                                     0.4 0.3 0;
%                                     0.4 0.4 0;
%                                     0.3 0.6 0;
%                                     0.25 0.55 0];
% limit.limite(2).surfaces.surface2=[0.3 0.3 0.5;
%                                     0.4 0.3 0.5;
%                                     0.4 0.4 0.5;
%                                     0.3 0.6 0.5;
%                                     0.25 0.55 0.5];
% limit.limite(2).rayonProxy=0.9;
% limit.limite(2).offset=[0.3 0.1 0.4];
% 
% %solide 3
% 
limit.limite(3).type='poly';
limit.limite(3).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(3).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(3).rayonProxy=0.6;
limit.limite(3).offset=[-0.35 -0.25 0];
limit.limite(3).opp=0.3;
% %solide 4
% 
limit.limite(4).type='poly';
limit.limite(4).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(4).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(4).rayonProxy=0.6;
limit.limite(4).offset=[-0.35 -0.35 0];
limit.limite(4).opp=0.3;

% %solide 5
% 
limit.limite(5).type='poly';
limit.limite(5).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(5).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(5).rayonProxy=0.6;
limit.limite(5).offset=[-0.35 -0.40 0];
limit.limite(5).opp=0.3;

% %solide 6
% 
% limit.limite(6).type='tube';
% limit.limite(6).surfaces.axe=[1 0 0]/norm([1 0 0]);
% limit.limite(6).surfaces.base=[0.4 0.4 0.2];
% limit.limite(6).surfaces.longueur=0.4;
% limit.limite(6).surfaces.dia=0.1;
% limit.limite(6).rayonProxy=limit.limite(6).surfaces.longueur;

% solide 6

limit.limite(6).type='poly';
limit.limite(6).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.25 -0.05;
                                    0 0.25 -0.05;
                                    0 0 -0.05];
limit.limite(6).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.25 0.05;
                                    0 0.25 0.05;
                                    0 0 0.05];
limit.limite(6).rayonProxy=0.6;
limit.limite(6).offset=[-0.35 -0.2 0];
limit.limite(6).opp=0.3;




%Définition des centroides de chaque solides
for i=1:length(limit.limite)
    if all(limit.limite(i).type=='tube')
        limit.limite(i).centroide=limit.limite(i).surfaces.axe*limit.limite(i).surfaces.longueur/2+limit.limite(i).surfaces.base;
    elseif all(limit.limite(i).type=='poly')
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
    if all(limit.limite(i).type=='poly')
        s=size(limit.limite(i).surfaces.surface1);
        if isequal(limit.limite(i).surfaces.surface1(:,1:2),limit.limite(i).surfaces.surface2(:,1:2))==0
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

%On enlève le solide qui a été discrétisé.

for i=1:length(dup)
    limit.limite(dup(i))=[];
end

%Positionnement des solides

for i=1:length(limit.limite)
    if all(limit.limite(i).type=='poly')
        s=size(limit.limite(i).surfaces.surface1);
        for j = 1:s(1)
            limit.limite(i).surfaces.surface1(j,:)=limit.limite(i).surfaces.surface1(j,:)-limit.limite(i).centroide+limit.limite(i).offset;
            limit.limite(i).surfaces.surface2(j,:)=limit.limite(i).surfaces.surface2(j,:)-limit.limite(i).centroide+limit.limite(i).offset;
        end
        limit.limite(i).centroide=limit.limite(i).offset;
    end
end

