function [Prisms, Cylinders, Spheres, Cones] = a_ProtectionZoneDef()

    Prisms=[];
    Cylinders=[];
    Spheres=[];
    Cones=[];

    Prisms(1,:,1)=[0.3,0.1,0,0,1,0];
    Prisms(2,:,1)=[0.4,-0.2,0,0,-1,0];
    Prisms(3,:,1)=[0.4,0.1,0,1,0,0];
    Prisms(4,:,1)=[0.3,-0.2,0,-1,0,0];

    Prisms(1,:,2)=[-0.8,0.2,0,1,1,0]+[0.4,0,0,0,0,0];
    Prisms(2,:,2)=[-0.8,-0.2,0,-1,-1,0]+[0.4,0,0,0,0,0];
    Prisms(3,:,2)=[-0.6,0,0,1,-1,0]+[0.4,0,0,0,0,0];
    Prisms(4,:,2)=[-1,0,0,-1,1,0]+[0.4,0,0,0,0,0];
    
end