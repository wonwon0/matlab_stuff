%test stl files

[v2, f2, n, c, stltitle] = stlread('stl_env_sim/test_poly_0.stl', 0);
p2=patch('Faces',f2,'Vertices',v2,'FaceVertexCData',c);
set(p2, 'facec', 'flat');  
set(p2, 'facec', [0 0.6 1]);