function write_limits(filename, limit)
addpath('../rrr version phil/geom3d/geom3d')
addpath('../rrr version phil/geom3d/meshes3d')
cd 'test_mass_spawn/'
delete *.stl
cd '..'
%afficheur de limites
n=length(limit.limite);
cmap = hsv(n);
X_global = [];
T = [1 2 4 3;1 2 6 5;5 6 8 7;3 4 8 7;2 4 8 6;1 3 7 5];
faces = triangulateFaces(T);
faces_global = [];

for i=1:n
    if all(limit.limite(i).type=='poly')
        pt_x=[limit.limite(i).surfaces.surface1(1:end-1,1);limit.limite(i).surfaces.surface2(1:end-1,1)];
        pt_y=[limit.limite(i).surfaces.surface1(1:end-1,2);limit.limite(i).surfaces.surface2(1:end-1,2)];
        pt_z=[limit.limite(i).surfaces.surface1(1:end-1,3);limit.limite(i).surfaces.surface2(1:end-1,3)];
        temp_x_1 = pt_x(3,:);
        temp_x_2 = pt_x(7,:);
        pt_x(3,:) = pt_x(4,:);
        pt_x(7,:) = pt_x(8,:);
        pt_x(4,:) = temp_x_1;
        pt_x(8,:) = temp_x_2;
        temp_y_1 = pt_y(3,:);
        temp_y_2 = pt_y(7,:);
        pt_y(3,:) = pt_y(4,:);
        pt_y(7,:) = pt_y(8,:);
        pt_y(4,:) = temp_y_1;
        pt_y(8,:) = temp_y_2;
        temp_z_1 = pt_z(3,:);
        temp_z_2 = pt_z(7,:);
        pt_z(3,:) = pt_z(4,:);
        pt_z(7,:) = pt_z(8,:);
        pt_z(4,:) = temp_z_1;
        pt_z(8,:) = temp_z_2;
        %T = delaunay(pt_x,pt_y,pt_z);
        X=[pt_x,pt_y,pt_z];
        n_points=size(X_global,1);
        faces_global = [faces_global;faces + n_points];
        X_global = [X_global;X];
        name = strcat('test_mass_spawn/',filename,'_',num2str(i),'.stl');
        stlwrite(name,faces,X)
        cd 'test_mass_spawn/'
        command = strcat({'meshlabserver -i '},filename,'_',num2str(i),{'.stl -o '},filename,'_',num2str(i),'.stl -s correct_normals.mlx');
        system(['export LD_LIBRARY_PATH="";' command{1}])
        cd '..'
    end
    
end
% name = strcat('stl_env_sim/',filename,'_0','.stl');
% stlwrite(name,faces_global,X_global)