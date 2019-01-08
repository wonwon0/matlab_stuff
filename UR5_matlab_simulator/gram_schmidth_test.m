function [perp_vectors, vectors_saved] = gram_schmidth_test(vectors)
% takes vectors and generates the set of vectors that are
% "perpendicular", or linearly independant to the inputs.
% The gram-schmidth process is used.
vectors = normc(vectors);
M_ident = eye(size(vectors,1));
u(:,1) = vectors(:,1);
vectors_t = vectors;
vectors = licols(vectors);
% for i = 1:size(vectors_t,2)
%     for j = (i+1):(size(vectors_t,2))
%         if i == j
%             continue
%         end
%         norm(project_vector(vectors_t(:,j),vectors_t(:,i)))
%         if norm(project_vector(vectors_t(:,j),vectors_t(:,i))) > 0.001 % clearing allmost linearly dependent vectors
%             break
%         end
%         vectors = [vectors vectors_t(:,j)];
%     end
% end
vectors_saved = vectors;
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
    if 0 % applying gram_schmidt method
        continue
    else
        u = [u M_ident(:,i) - project_vector(u,M_ident(:,i))];  
    end
end
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

function [Xsub,idx]=licols(X,tol)
%Extract a linearly independent set of columns of a given matrix X
%
%    [Xsub,idx]=licols(X)
%
%in:
%
%  X: The given input matrix
%  tol: A rank estimation tolerance. Default=1e-10
%
%out:
%
% Xsub: The extracted columns of X
% idx:  The indices (into X) of the extracted columns

     if ~nnz(X) %X has no non-zeros and hence no independent columns

         Xsub=[]; idx=[];
         return
     end

     if nargin<2, tol=1e-10; end

       [Q, R, E] = qr(X,0); 

       if ~isvector(R)
        diagr = abs(diag(R));
       else
        diagr = R(1);   
       end

       %Rank estimation
       r = find(diagr >= tol*diagr(1), 1, 'last'); %rank estimation

       idx=sort(E(1:r));

       Xsub=X(:,idx);                      
