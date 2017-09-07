function [Limitation, inside, m] = a_ProtectionZonePrisms3D_elbow(posActu, theta, Prisms, gap, Limitation)

inside = 0;
m=0;

    if size(Prisms,1)>0
      for i=1:size(Prisms,3)
            inZone = 1;
            D=[];
            for j=1:size(Prisms,1)
                n = Prisms(j,4:6,i)';
                n=n/norm(n);
                w = posActu - (Prisms(j,1:3,i)'+gap*n);
                d = (n'*w);
                D(j,:)=[abs(n'*w),n'];
                if d > 0 
                    inZone = 0;
                end
            end
            if inZone == 1
                [m,index] = min(D(:,1));
                normal = D(index,2:4)';

                [Ja] = a_Jacobian_elbow(theta); % Jacobienne du coude
                [J, Jinv] = a_Jacobian(theta);  % Jacobienne de effecteur

                Jaj=Ja*Jinv; % Multiplication des jacobiennes
                normal_elbow=[(normal(1:3)'*Jaj)]'; % Normale modifiee

                Limitation = [Limitation,normal_elbow];
                inside=1;
            end
        end
    end
end