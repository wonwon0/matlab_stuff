function membrures_robot = get_membrures_robot()

    membrures_robot = containers.Map;


    membrures_robot('ur_on_table::ur_1_shoulder_limb::ur_1_shoulder_limb_collision') = 1;
    membrures_robot('ur_on_table::ur_2_upperarm_limb::ur_2_upperarm_limb_collision') = 2;
    membrures_robot('ur_on_table::ur_3_forearm_limb::ur_3_forearm_limb_collision') = 3;
    membrures_robot('ur_on_table::ur_4_upperwrist_limb::ur_4_upperwrist_limb_collision') = 4;
    membrures_robot('ur_on_table::ur_5_lowerwrist_limb::ur_5_lowerwrist_limb_collision') = 5;
    membrures_robot('ur_on_table::ur_6_hand_limb::ur_6_hand_limb_collision') = 6;
    membrures_robot('ur_on_table::ur_6_hand_limb::ur_6_hand_limb_fixed_joint_lump__ur_ee_link_collision_1') = 6;
end