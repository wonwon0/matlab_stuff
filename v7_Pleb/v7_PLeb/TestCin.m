clc
close all
clear all

theta = [0;-45;-90]*pi/180
[T,T1,T2,T3] = a_CinDirecte(theta);
T


[thetaout,error] = a_InvKinematics(theta,[T(1,4);T(2,4);T(3,4)]);
thetaout
error

[T,T1,T2,T3] = a_CinDirecte(thetaout);
T

[J, Jinv] = a_Jacobian(theta);
J