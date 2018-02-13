    
function [normale_effecteur, collision_pose_eff] = collision_manager(contact_subscriber, Robot_Pose_j, Robot_Poses, dh_eff)


contacts = receive(contact_subscriber,10);
normale_effecteur = []; collision_pose = [];

membrures_robot = containers.Map;


membrures_robot('ur_on_table::ur_1_shoulder_limb::ur_1_shoulder_limb_collision') = 1;
membrures_robot('ur_on_table::ur_2_upperarm_limb::ur_2_upperarm_limb_collision') = 2;
membrures_robot('ur_on_table::ur_3_forearm_limb::ur_3_forearm_limb_collision') = 3;
membrures_robot('ur_on_table::ur_4_upperwrist_limb::ur_4_upperwrist_limb_collision') = 4;
membrures_robot('ur_on_table::ur_5_lowerwrist_limb::ur_5_lowerwrist_limb_collision') = 5;
membrures_robot('ur_on_table::ur_6_hand_limb::ur_6_hand_limb_collision') = 6;
membrures_robot('ur_on_table::ur_6_hand_limb::ur_6_hand_limb_fixed_joint_lump__ur_ee_link_collision_1') = 6;

z_offset = 0.899734982808; % pour tenir compte de l'élévation du robot dans le simulateur

t = 1;
for i=1:size(Robot_Poses,1)
    jacob_eff = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh_eff);
    for j=1:length(contacts.Contacts)
        membrure = membrures_robot(contacts.Contacts(j).Collision2);
        
        
        Pose_base_membrure = cin_dir_6ddl( Robot_Pose_j, dh_UR5(membrure-1));
        Pose_base_membrure = Pose_base_membrure(1:3,4)';
        Pose_bout_membrure = cin_dir_6ddl( Robot_Pose_j, dh_UR5(membrure));
        Pose_bout_membrure = Pose_bout_membrure(1:3,4)';
        Pose_collision = (contacts.Contacts(j).Position(1:3)' - [0, 0, z_offset]) * 1000;
        
        Pose_proj_collision = projPointOnLine3d(Pose_collision, [Pose_base_membrure,Pose_bout_membrure]);
        
        
        ratio = norm(Pose_proj_collision - Pose_base_membrure) / norm(Pose_bout_membrure - Pose_base_membrure);
        
        dh_local = dh_UR5(membrure, ratio);
        jacob_local = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh_local);
        
        normal = [-contacts.Contacts(j).Normal(1), -contacts.Contacts(j).Normal(2), -contacts.Contacts(j).Normal(3)];
        normale_effecteur_loc=PointToEffector_v2(normal, jacob_local, jacob_eff );
        normale_effecteur(t,:) = [normale_effecteur; normale_effecteur_loc(1,1:3)];
        collision_pose_eff(t,:) = Robot_Poses(1, :) - normale_effecteur_loc(1,1:3);
        Pose_collision = collision_pose - normal;
        
        d_min(t) = 0;
        t = t+1;
    end
end

















