function [Limitation,inside,m] = a_ProtectionBase3D(posActu,Limitation,Base_H1,Base_H2,Base_H3,Base_H4,Base_Cyl1,Hard_cylindreInt)

        inside = 0;
        m=0;


        posZ = posActu(3);

        % Si dans le cylindre autour de la base
        if posZ >= Base_H2 && posZ<= Base_H3
            a = [0;0;1];
            b = posActu - [0;0;Base_H2];
            n = b-(b'*a)*a;
            d = norm(n);
            if d < 0.001
                n=[1;0;0];
                if (n'*a) < 0.001
                    n=[0;1;0];
                end
            end
            if d <= Base_Cyl1
                inside = 1; % A linterieur du cylindre
                m = abs(Base_Cyl1 - d); %Distance a linterieur du cylindre
                Limitation = [Limitation,n/norm(n)];
            end
            
        elseif posZ >= Base_H3 && posZ<= Base_H4 % Cone superieur
            Z0 = (Base_Cyl1*Base_H4-Hard_cylindreInt*Base_H3)/(Base_Cyl1-Hard_cylindreInt); % Voir les equations du cone
            b = posActu - [0;0;Z0];
            a = [0;0;1];
            c = (b'*a)*a;
            e = [Base_Cyl1;0;Base_H3]-[0;0;Z0];
            e = e/norm(e);
            thetac = acos(e'*-a);
            f = c*tan(thetac);
            d = b - (b'*a)*a;
            if norm(d) <= norm(f)
                inside = 1;
                m = norm(d) - norm(f);
                t=cross(b,a)/norm(b);
                n = cross(t,b)/norm(b);
                Limitation = [Limitation,n/norm(n)];
            end
            
        elseif posZ >= Base_H1 && posZ<= Base_H2 % Cone inferieur
            Z0 = (Base_Cyl1*Base_H1-Hard_cylindreInt*Base_H2)/(Base_Cyl1-Hard_cylindreInt);
            b = posActu - [0;0;Z0];
            a = [0;0;-1];
            c = (b'*a)*a;
            e = [Base_Cyl1;0;Base_H2]-[0;0;Z0];
            e = e/norm(e);
            thetac = acos(e'*-a);
            f = c*tan(thetac);
            d = b - (b'*a)*a;
            if norm(d) <= norm(f)
                inside = 1;
                m = norm(d) - norm(f);
                t=cross(b,a)/norm(b);
                n = cross(t,b)/norm(b);
                Limitation = [Limitation,n/norm(n)];
            end
            
        else % Autre: cylindre de singularite
            a = [0;0;1];
            b = posActu - [0;0;-100];
            n = b-(b'*a)*a;
            d = norm(n);
            if d < 0.001
                n=[1;0;0];
                if (n'*a) < 0.001
                    n=[0;1;0];
                end
            end
            if d <= Hard_cylindreInt
                inside = 1;
                m = abs(Hard_cylindreInt - d);
                Limitation = [Limitation,n/norm(n)];
            end
        end