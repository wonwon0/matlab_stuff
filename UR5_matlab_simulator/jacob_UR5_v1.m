function [ jac ] = jacob_UR5_v1( theta_act, Pose, dh, membrure)
    %DISCLAIMER!
    %Le code de cette fonction a été en partie généré automatiquement par
    %maple. Se référer au fichier UR5_all_joints.mw dans le répertoire
    %phleb38/calcul_robot/UR5/ sur le drive du labo de robotique.


    %theta_act: position angulaire des joint du robot
    %pose: position et orientation de l'effecteur
    %dh: structure contenant les param. dh du robot
    if nargin < 4
        membrure = 6;
    end
    if length(Pose)>3
        lol = 1;
    end
    theta=theta_act;
    x=Pose(1);y=Pose(2);z=Pose(3);
    s = sin(theta_act);
    c = cos(theta_act);
    [v1, v2, v3, v4, v5, v6] = vitesse_cart_jac_UR5(theta, dh);
    jac=[ 0 s(1) s(1) s(1) c(1)*sin(theta(2)+theta(3)+theta(4)) -c(1)*cos(theta(2)+theta(3)+theta(4))*s(5)+s(1)*c(5);
        0 -c(1) -c(1) -c(1) s(1)*sin(theta(2)+theta(3)+theta(4)) -s(1)*cos(theta(2)+theta(3)+theta(4))*s(5)-c(1)*c(5);
        1 0 0 0 -cos(theta(2)+theta(3)+theta(4)) -sin(theta(2)+theta(3)+theta(4))*s(5);
        v1 v2 v3 v4 v5 v6];
end

function [v1, v2, v3, v4, v5, v6] = vitesse_cart_jac_UR5(theta, dh)
l(1) = dh.b(1); l(2) = dh.b(2); l(3) = dh.a(2); l(4) = dh.b(3);
l(5) = dh.a(3); l(6) = dh.b(4); l(7) = dh.b(5); l(8) = dh.b(6);
    v1 = vitesse_v1(theta, l);
    v2 = vitesse_v2(theta, l);
    v3 = vitesse_v3(theta, l);
    v4 = vitesse_v4(theta, l);
    v5 = vitesse_v5(theta, l);
    v6 = vitesse_v6(theta, l);
end
function v1 = vitesse_v1(theta, l)

t1 = sin(theta(1));
t2 = cos(theta(2));
t3 = cos(theta(1));
t4 = cos(theta(3));
t5 = sin(theta(2));
t6 = sin(theta(3));
t7 = t5 * t6;
t8 = t2 * t4 - t7;
t9 = t1 * t8;
t10 = sin(theta(4));
t5 = t2 * t6 + t4 * t5;
t6 = t1 * t5;
t11 = cos(theta(4));
t12 = sin(theta(5));
t13 = cos(theta(5));
t2 = t2 * (t4 * l(5) + l(3)) - t7 * l(5);
t4 = -l(2) - l(4) - l(6);
t7 = t3 * t8;
t5 = t3 * t5;
cg = [-t1 * t2 - t3 * t4 - (t10 * t9 + t11 * t6) * l(7) + (t3 * t13 + (-t10 * t6 + t11 * t9) * t12) * l(8) -t1 * t4 + t2 * t3 + (t10 * t7 + t11 * t5) * l(7) - (-t1 * t13 + t12 * (-t10 * t5 + t11 * t7)) * l(8) 0];

v1 = cg';
end
function v2 = vitesse_v2(theta, l)
t1 = cos(theta(1));
t2 = sin(theta(2));
t3 = cos(theta(3));
t4 = cos(theta(2));
t5 = sin(theta(3));
t6 = t4 * t5;
t7 = t2 * t3 + t6;
t8 = sin(theta(4));
t5 = t2 * t5;
t9 = -t3 * t4 + t5;
t10 = cos(theta(4));
t11 = sin(theta(5));
t3 = t3 * l(5) + l(3);
t2 = t2 * t3 + (t10 * t9 + t7 * t8) * l(7) - (t10 * t7 - t8 * t9) * t11 * l(8) + t6 * l(5);
t6 = sin(theta(1));
t12 = -t6 * t9;
t13 = t6 * t7;
t14 = cos(theta(5));
t3 = t3 * t4 - t5 * l(5);
t4 = -l(2) - l(4) - l(6);
t5 = -t1 * t9;
t7 = t1 * t7;
cg0 = [-t1 * t2 -t6 * t2 t1 * (t1 * t3 - t4 * t6 + (t10 * t7 + t5 * t8) * l(7) - (t11 * (t10 * t5 - t7 * t8) - t14 * t6) * l(8)) + t6 * (t1 * t4 + t6 * t3 + (t10 * t13 + t12 * t8) * l(7) - (t1 * t14 + t11 * (t10 * t12 - t13 * t8)) * l(8))];

v2 = cg0';
end
function v3 = vitesse_v3(theta, l)
t1 = cos(theta(1));
t2 = sin(theta(2));
t3 = cos(theta(3));
t4 = cos(theta(2));
t5 = sin(theta(3));
t6 = t2 * t3 + t4 * t5;
t7 = sin(theta(4));
t2 = t2 * t5 - t3 * t4;
t3 = cos(theta(4));
t4 = sin(theta(5));
t5 = t6 * l(5) + (t2 * t3 + t6 * t7) * l(7) - (-t2 * t7 + t3 * t6) * t4 * l(8);
t8 = sin(theta(1));
t9 = t8 * t2;
t10 = t8 * t6;
t11 = cos(theta(5));
t12 = l(4) + l(6);
t2 = t1 * t2;
t6 = t1 * t6;
cg1 = [-t1 * t5 -t8 * t5 t1 * (t12 * t8 + (-t2 * t7 + t3 * t6) * l(7) - (-t11 * t8 + t4 * (-t2 * t3 - t6 * t7)) * l(8) - t2 * l(5)) - t8 * (t1 * t12 - (t10 * t3 - t7 * t9) * l(7) + (t1 * t11 + t4 * (-t10 * t7 - t3 * t9)) * l(8) + t9 * l(5))];

v3 = cg1';

end
function v4 = vitesse_v4(theta, l)
t1 = cos(theta(1));
t2 = sin(theta(2));
t3 = cos(theta(3));
t4 = cos(theta(2));
t5 = sin(theta(3));
t6 = t2 * t3 + t4 * t5;
t7 = sin(theta(4));
t2 = t2 * t5 - t3 * t4;
t3 = cos(theta(4));
t4 = sin(theta(5));
t5 = -(t2 * t3 + t6 * t7) * l(7) + (-t2 * t7 + t3 * t6) * t4 * l(8);
t8 = sin(theta(1));
t9 = -t8 * t2;
t10 = t8 * t6;
t11 = cos(theta(5));
t2 = -t1 * t2;
t6 = t1 * t6;
cg2 = [t1 * t5 t8 * t5 t1 * (t8 * l(6) + (t2 * t7 + t3 * t6) * l(7) - (-t11 * t8 + t4 * (t2 * t3 - t6 * t7)) * l(8)) - t8 * (t1 * l(6) - (t10 * t3 + t7 * t9) * l(7) + (t1 * t11 + t4 * (-t10 * t7 + t3 * t9)) * l(8))];

v4 = cg2';

end
function v5 = vitesse_v5(theta, l)
t1 = sin(theta(1));
t2 = cos(theta(2));
t3 = cos(theta(3));
t4 = sin(theta(2));
t5 = sin(theta(3));
t6 = t2 * t3 - t4 * t5;
t7 = t1 * t6;
t8 = sin(theta(4));
t2 = t2 * t5 + t3 * t4;
t3 = t1 * t2;
t4 = cos(theta(4));
t5 = t3 * t4 + t7 * t8;
t9 = t2 * t8 - t4 * t6;
t10 = sin(theta(5));
t11 = -t9 * l(7) + (t2 * t4 + t8 * t6) * t10 * l(8);
t12 = cos(theta(1));
t13 = cos(theta(5));
t3 = t5 * l(7) - (t10 * (-t3 * t8 + t4 * t7) + t12 * t13) * l(8);
t6 = t12 * t6;
t2 = t12 * t2;
t7 = t2 * t4 + t8 * t6;
t1 = t7 * l(7) - (-t1 * t13 + t10 * (-t2 * t8 + t4 * t6)) * l(8);
cg3 = [-t11 * t5 - t3 * t9 t1 * t9 + t11 * t7 -t1 * t5 + t3 * t7];

v5 = cg3';

end
function v6 = vitesse_v6(theta, l)
cg4 = [0 0 0];
v6 = cg4';
end






























