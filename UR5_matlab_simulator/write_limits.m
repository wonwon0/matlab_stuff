function write_limits(filename, limit)
%afficheur de limites
n=length(limit.limite);
cmap = hsv(n);
for i=1:n
    if all(limit.limite(i).type=='poly')
        pt_x=[limit.limite(i).surfaces.surface1(1:end-1,1);limit.limite(i).surfaces.surface2(1:end-1,1)];
        pt_y=[limit.limite(i).surfaces.surface1(1:end-1,2);limit.limite(i).surfaces.surface2(1:end-1,2)];
        pt_z=[limit.limite(i).surfaces.surface1(1:end-1,3);limit.limite(i).surfaces.surface2(1:end-1,3)];
        %T = delaunay(pt_x,pt_y,pt_z);
        X=[pt_x,pt_y,pt_z];
        T = [1 2 4 3;1 2 6 5;5 6 8 7;3 4 8 7;2 4 8 6;1 3 7 5;];
        faces = triangulateFaces(T);
        
        
        name = strcat('stl_env_sim/',filename,'_',num2str(i),'.stl');
        stlwrite(name,faces,X)
    end
end