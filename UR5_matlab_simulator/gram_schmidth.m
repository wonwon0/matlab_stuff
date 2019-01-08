function [perp_vectors] = gram_schmidth(vectors)
% takes vectors and generates the set of vectors that are
% "perpendicular", or linearly independant to the inputs.
% The gram-schmidth process is used.
vectors = normc(vectors);
M_ident = eye(size(vectors,1));
u(:,1) = vectors(:,1);
vectors_t = vectors;
if size(vectors,2) > 5
    vectors = [];
    for i = 1:size(vectors_t,2)
        for j = 1:size(vectors_t,2)
            if i == j
                continue
            end
            if norm(project_vector(vectors_t(:,j),vectors_t(:,i))) > 0.9999 % clearing allmost linearly dependent vectors
                break
            end
            vectors = [vectors vectors_t(:,i)];
        end
    end
end

if size(vectors,2) > 5 %no solutions for 6 DOF robot (6 non-linearly dependant vectors)
    perp_vectors = [];
    return
end
for i = 2:size(vectors,2)
    if sum(M_ident(:,i) - project_vector(u,M_ident(:,i))) == 0
        continue
    else
        u = [u vectors(:,i) - project_vector(vectors(:,1:i-1), vectors(:,i))];
    end
end

for i = 1:size(vectors,1)
    if sum(M_ident(:,i) - project_vector(u,M_ident(:,i))) == 0 % applying gram_schmidt method
        continue
    else
        u = [u M_ident(:,i) - project_vector(u,M_ident(:,i))];
    end
end
u
u(abs(u)<0.0000001) = 0; % clearing small values
vect_ok = [];
flag = 0;
for i = 1:size(u,2)
    for j = 1:size(vectors,2)
        if norm(project_vector(vectors(:,j),u(:,i))) > 0.99 % clearing allmost linearly dependent vectors
            flag = 1;
            break
        end
    end
    if flag == 1
        flag = 0;
        continue
    end
    if norm(u(:,i)) < 0.3 % clearing trivial solutions
        continue
    end
%     if any(isnan(u(:,i)))
%         continue
%     end
    vect_ok = [vect_ok u(:,i) / norm(u(:,i))];
end
perp_vectors = vect_ok;
