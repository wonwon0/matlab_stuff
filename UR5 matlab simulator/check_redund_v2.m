function [ vec_norm ] = check_redund_v3(pose_act, vec_norm_red,d_min,pose_prox)
    %fonction d�tectant les vecteurs normaux redondants. Si deux points
    %proximaux forment un vecteur qui est perpendiculaire � une des
    %deux normales attach�es � ces points, un des deux points forme alors
    %une normale � l'effecteur redondante. Ceci cause un probl�me lorsque
    %on veut slider sur des joints.
    %La fonction �limine aussi les limites qui sont cach�es en arri�res
    %d'autres limites.
    t=pose_prox;
    remove=[];
    [vec_norm c d]=unique(vec_norm_red,'rows','stable');
    for i=1:size(vec_norm,1)
        for j=1:6
            if abs(vec_norm(i,j))*4<mean(abs(vec_norm(i,:)))
            vec_norm(i,j)=0;
            end
        end
        vec_norm(i,:)=vec_norm(i,:)/norm(vec_norm(i,:));
    end
    vec_norm(~any(isnan(vec_norm),2),:);
    d_min=d_min(c);
    n=size(vec_norm,1);

    for i=1:n
        for j=i+1:n
            point_1=pose_act-vec_norm(i,:)*d_min(i);
            point_2=pose_act-vec_norm(j,:)*d_min(j);
            vec_12=(point_2-point_1)/norm(point_2-point_1);
            d1 = -sum(bsxfun(@times, vec_norm(j,:), bsxfun(@minus, point_2, point_1)), 2);
            d2 = -sum(bsxfun(@times, vec_norm(i,:), bsxfun(@minus, point_1, point_2)), 2);
            if any(isnan(vec_12))
                dbstop at 15 in check_redund;
            end
            a=norm(vec_12*vec_norm(i,:)');
            b=norm(vec_12*vec_norm(j,:)');
            if a<0.00001 || b<0.00001
                if d_min(i)>d_min(j)
                    remove=[remove i];
                    i=i-1;
                    break
                else
                    remove=[remove j];
                end
            elseif a>0.9999 && b>0.9999
                if norm(point_2-point_1)<0.001
                    if d_min(i)>d_min(j)
                        remove=[remove i];
                        i=i-1;
                        break
                    else
                        remove=[remove j];
                    end
                end
            elseif d2<0
                remove=[remove j];
            elseif d1<0
                remove=[remove i];
                i=i-1;
                break
            end
        end
        
        vec_norm(remove,:)=[];
        d_min(remove)=[];
        remove=[];
        n=size(vec_norm,1);
    end
end

