function [Prisms, Cylinders, Spheres, Cones] = a_ProtectionZoneDef_3D()

    Prisms=[];
    Cylinders=[];
    Spheres=[];
    Cones=[];

    
    
    x = [-0.14 0.14];
    y = [0.28 0.45];
    z = [0.2 0.5];
    
%     x = [-0.14 0.14];
%     y = [0.1 0.45];
%     z = [0.2 0.4];
    
    % Pour un prisme aligne avec les axes XYZ
    Prisms(1,:,1)=[x(1), y(1), z(1),0,-1,0]; % Position d'un coin de chaque face et de la normale
    Prisms(2,:,1)=[x(1), y(2), z(1),0,1,0];
    Prisms(3,:,1)=[x(1), y(2), z(1),-1,0,0];
    Prisms(4,:,1)=[x(2), y(2), z(1),1,0,0];
    Prisms(5,:,1)=[x(2), y(2), z(1),0,0,-1];
    Prisms(6,:,1)=[x(2), y(2), z(2),0,0,1];
    
%     Prisms(1,:,1)=[x(1), y(1), z(1),0,-1,0];
%     Prisms(2,:,1)=[x(2), y(1), z(1),0,-1,0];
%     Prisms(3,:,1)=[x(2), y(1), z(2),0,-1,0];
%     Prisms(4,:,1)=[x(1), y(1), z(2),0,-1,0];
% 
%     Prisms(1,:,2)=[x(1), y(2), z(1),0,1,0];
%     Prisms(2,:,2)=[x(2), y(2), z(1),0,1,0];
%     Prisms(3,:,2)=[x(2), y(2), z(2),0,1,0];
%     Prisms(4,:,2)=[x(1), y(2), z(2),0,1,0];
%     
%     Prisms(1,:,3)=[x(1), y(1), z(1),-1,0,0];
%     Prisms(2,:,3)=[x(1), y(2), z(1),-1,0,0];
%     Prisms(3,:,3)=[x(1), y(1), z(2),-1,0,0];
%     Prisms(4,:,3)=[x(1), y(2), z(2),-1,0,0];
%     
%     Prisms(1,:,4)=[x(2), y(1), z(1),1,0,0];
%     Prisms(2,:,4)=[x(2), y(2), z(1),1,0,0];
%     Prisms(3,:,4)=[x(2), y(1), z(2),1,0,0];
%     Prisms(4,:,4)=[x(2), y(2), z(2),1,0,0];
%     
%     Prisms(1,:,5)=[x(1), y(1), z(1),0,0,1];
%     Prisms(2,:,5)=[x(2), y(1), z(1),0,0,1];
%     Prisms(3,:,5)=[x(1), y(2), z(1),0,0,1];
%     Prisms(4,:,5)=[x(2), y(2), z(1),0,0,1];
%     
%     Prisms(1,:,6)=[x(1), y(1), z(2),0,0,-1];
%     Prisms(2,:,6)=[x(2), y(1), z(2),0,0,-1];
%     Prisms(3,:,6)=[x(1), y(2), z(2),0,0,-1];
%     Prisms(4,:,6)=[x(2), y(2), z(2),0,0,-1];
    
    
    
end