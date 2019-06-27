function test_mode = interactive_test_solids(pose_eff, solid_to_eval)
% vérifie si l'effecteur est dans la bonne position/orrientation pour
% déterminer si l'objectif du test est atteint. Supprime le modèle gazebo
% du solide à atteinfre si c'est le cas.
% solid_to_eval et le numero du solide (1 ou 2) qu'on évalue.
lib_export = 'export LD_LIBRARY_PATH="/home/phil/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";';
test_mode = 1;
    if (abs(pose_eff(3)-535) < 50) && (abs(pose_eff(2)-510)) < 50 && (solid_to_eval == 1)
        spawn_model_1 = create_model_spawn_command('test_mass_spawn/', 1, [0 1 0],0.0011);
        system([lib_export spawn_model_1]);
        test_mode = 0;
    elseif (abs(pose_eff(3)-7) < 100) && (abs(pose_eff(2)+182)) < 10 && (solid_to_eval == 2)
        spawn_model_2 = create_model_spawn_command('test_mass_spawn/', 2, [0 1 0],0.0011);
        system([lib_export spawn_model_2]);
        test_mode = 0;
    elseif (abs(pose_eff(3)-925) < 100) && (abs(pose_eff(1)+385)) < 10 && (solid_to_eval == 3)
        spawn_model_3 = create_model_spawn_command('test_mass_spawn/', 3, [0 1 0],0.0011);
        system([lib_export spawn_model_3]);
        test_mode = 0;
    elseif solid_to_eval == 10
        delete_model_1 = 'rosservice call /gazebo/delete_model "model_name: ''model_1''"';
        system([lib_export delete_model_1]);
    elseif solid_to_eval == 20
        delete_model_2 = 'rosservice call /gazebo/delete_model "model_name: ''model_2''"';
        system([lib_export delete_model_2]);
    elseif solid_to_eval == 30
        delete_model_3 = 'rosservice call /gazebo/delete_model "model_name: ''model_3''"';
        system([lib_export delete_model_3]);
    end
end

