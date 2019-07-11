# matlab_stuff
all my unmaintained matlab code, some written in french, other in english, venture at you own risk!

Prérequis pour le lancement du simulateur :
 Installation de Matlab avec le package robotics (pour communiquer avec ROS)
 Installation de ROS.
 
 Il faut ensuite créer un catkin warkspace et cloner ce dépot github ainsi que d'autres dépots github:
 - https://github.com/wonwon0/ur-arm-pkgs
 - https://github.com/wonwon0/joint-control-pkgs
 - https://github.com/wonwon0/convenience-pkgs
 
 Ensuite effectuer une catkin_make.

Pour lancer la simulation 6DDL, ouvrir matlab, naviguer dans le répertoire UR5_matalb_simulator et lancer la fonction:

launch_sim('sim_6ddl_matlab_v2.m', true, true,2)

L'algorithme de gram schmidt est implémenté dans le fichier gram_schmidt.m dans le même répertoire.
