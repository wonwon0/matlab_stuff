function interactive_test_solids(pose_eff, gazebo)
% vérifie si l'effecteur est dans la bonne position/orrientation pour
% déterminer si l'objectif du test est atteint. Supprime le modèle gazebo
% du solide à atteinfre si c'est le cas.

    if abs(pose_eff(3)-535) < 50 && abs(pose_eff(2)-510) < 50
        a = 'pouet'
        if ismember('test_poly_1',getSpawnedModels(gazebo))
            removeModel(gazebo,'test_poly_1');
        end
    end
end

