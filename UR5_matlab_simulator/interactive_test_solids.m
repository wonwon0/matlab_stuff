function test_mode = interactive_test_solids(pose_eff, solid_to_eval)
% vérifie si l'effecteur est dans la bonne position/orrientation pour
% déterminer si l'objectif du test est atteint. Supprime le modèle gazebo
% du solide à atteinfre si c'est le cas.
% solid_to_eval et le numero du solide (1 ou 2) qu'on évalue.
    if abs(pose_eff(3)-535) < 50 && abs(pose_eff(2)-510) < 50 && solid_to_eval == 1
        system(['export LD_LIBRARY_PATH="/home/phil/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu";' 'rosservice call /gazebo/delete_model "model_name: ''test_poly_1''"']);
        test_mode = 0;
    else
        test_mode = 1;
    end
end

