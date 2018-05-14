function [proj_vector] = project_vector(matrix_vectors,vector_2)
%projects vector_2 on all column vectors of matrix_vectors
proj_vector = zeros(length(vector_2),1);

for i =1:size(matrix_vectors,2)
    if any(isnan((matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2))
        continue
    end
    proj_vector = proj_vector + (matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2;
end

