function limit=StructureLimites_v2()
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
limit.limite(1).surfaces.surface1=[0 0 0;
                                    0.35 0 0;
                                    0.35 0.28 0;
                                    0 0.28 0;
                                    0 0 0];
limit.limite(1).surfaces.surface2=[0 0 0.48;
                                    0.35 0 0.48;
                                    0.35 0.28 0.48;
                                    0 0.28 0.48;
                                    0 0 0.48];
limit.limite(1).rayonProxy=0.6;
limit.limite(1).offset=[0.5 -0.2 0.16];
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

limit.limite(2).surfaces.surface2=[0 0 0.3;
                                    1.5 0 0.3;
                                    1.5 1.5 0.3;
                                    0 1.5 0.3;
                                    0 0 0.3];
limit.limite(2).surfaces.surface1=[0 0 0;
                                    1.5 0 0;
                                    1.5 1.5 0;
                                    0 1.5 0;
                                    0 0 0];
limit.limite(2).rayonProxy=1;
limit.limite(2).offset=[0 0 -0.2];
limit.limite(2).opp=1;

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
limit.limite(3).surfaces.surface1=[-0.08 -0.47 -0.05;
                                    0.04 -0.47 -0.05;
                                    0.04 -0.16 -0.05;
                                    -0.08 -0.16 -0.05;
                                    -0.08 -0.47 -0.05];
limit.limite(3).surfaces.surface2=[-0.08 -0.47 0.08;
                                    0.04 -0.47 0.08;
                                    0.04 -0.16 0.08;
                                    -0.08 -0.16 0.08;
                                    -0.08 -0.47 0.08];
limit.limite(3).rayonProxy=sqrt(3*0.1^2)+0.4;
limit.limite(3).opp=0.3;
% %solide 4
% 
limit.limite(4).type='poly';
limit.limite(4).surfaces.surface1=[0 -0.47 -0.05;
                                    0.4 -0.47 -0.05;
                                    0.4 -0.16 -0.05;
                                    0 -0.16 -0.05;
                                    0 -0.47 -0.05];
limit.limite(4).surfaces.surface2=[0 -0.47 0.08;
                                    0.4 -0.47 0.08;
                                    0.4 -0.16 0.08;
                                    0 -0.16 0.08;
                                    0 -0.47 0.08];
limit.limite(4).rayonProxy=sqrt(3*0.1^2)+0.4;
limit.limite(4).opp=0.3;

% %solide 5
% 
limit.limite(5).type='tube';
limit.limite(5).surfaces.axe=[0 0 1]/norm([0 0 1]);
limit.limite(5).surfaces.base=[0 0 0];
limit.limite(5).surfaces.longueur=1.5;
limit.limite(5).surfaces.dia=0.12;
limit.limite(5).rayonProxy=limit.limite(5).surfaces.longueur;
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

%solide 7

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

limit.limite(7).type='poly';
limit.limite(7).surfaces.surface1=[0 0 -0.05;
                                    0.1 0 -0.05;
                                    0.1 0.35 -0.05;
                                    0 0.35 -0.05;
                                    0 0 -0.05];
limit.limite(7).surfaces.surface2=[0 0 0.05;
                                    0.1 0 0.05;
                                    0.1 0.35 0.05;
                                    0 0.35 0.05;
                                    0 0 0.05];
limit.limite(7).rayonProxy=0.6;
limit.limite(7).offset=[-0.495 -0.2 0];
limit.limite(7).opp=0.3;
%solide 8

limit.limite(8).type='sphe';
limit.limite(8).radius=0.65;
limit.limite(8).rayonProxy=1;
limit.limite(8).centroide=[0 0 0.2755];
limit.limite(8).opp=0;
%solide 9

limit.limite(9).type='poly';
limit.limite(9).surfaces.surface1=[0 0 0;
                                    0 0.3 0;
                                    0.3 0.3 0;
                                    0 0 0];
limit.limite(9).surfaces.surface2=[0 0 0.5;
                                    0 0.3 0.5;
                                    0.3 0.3 0.5;
                                    0 0 0.5];
limit.limite(9).rayonProxy=0.4;
limit.limite(9).offset=[0.2 -0.18 0.1];
limit.limite(9).opp=0.3;

%solide 10

limit.limite(10).type='sphe';
limit.limite(10).radius=0.1;
limit.limite(10).rayonProxy=1;
limit.limite(10).centroide=[0 0.2 0.2755];
limit.limite(10).opp=0.3;

%solide 11

limit.limite(11).type='sphe';
limit.limite(11).radius=0.1;
limit.limite(11).rayonProxy=1;
limit.limite(11).centroide=[0 0.2 0.2755+0.239];
limit.limite(11).opp=0.3;

% % %solide 10
% % 
% % limit.limite(10).type='tube';
% % limit.limite(10).surfaces.axe=[0 0 1]/norm([0 0 1]);
% % limit.limite(10).surfaces.base=[-0.25 -0.47 -0.07];
% % limit.limite(10).surfaces.longueur=0.25;
% % limit.limite(10).surfaces.dia=0.05;
% % limit.limite(10).rayonProxy=limit.limite(10).surfaces.longueur;
% % limit.limite(10).opp=1;
% % 
% % %solide 11
% % 
% % limit.limite(11).type='tube';
% % limit.limite(11).surfaces.axe=[0 0 1]/norm([0 0 1]);
% % limit.limite(11).surfaces.base=[-0.38 -0.43 -0.07];
% % limit.limite(11).surfaces.longueur=0.25;
% % limit.limite(11).surfaces.dia=0.05;
% % limit.limite(11).rayonProxy=limit.limite(11).surfaces.longueur;
% % limit.limite(11).opp=1;


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
limit.limite(3).offset=limit.limite(3).centroide;
limit.limite(4).offset=limit.limite(4).centroide+[-0.4 0 0.4];
limit.limite(5).offset=limit.limite(5).centroide+[0 0 -0.05];
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

