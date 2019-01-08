function [proj_vector] = project_vector(matrix_vectors,vector_2)
%projects vector_2 on all columns vectors of matrix_vectors
proj_vector = zeros(length(vector_2),1);
if all(vector_2 == [1 0 0 0 0 0 ]')
    for i =1:size(matrix_vectors,2)
        if any(isnan((matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2))
            continue
        end
        
        a = (matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2
        proj_vector = proj_vector + (matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2;
    end
    proj_vector
else
    for i =1:size(matrix_vectors,2)
        if any(isnan((matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2))
            continue
        end
        proj_vector = proj_vector + (matrix_vectors(:,i)'*vector_2 * matrix_vectors(:,i)) / norm(matrix_vectors(:,i))^2;
    end
end

