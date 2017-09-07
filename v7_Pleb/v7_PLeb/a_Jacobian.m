function [J, Jinv] = a_Jacobian(theta)

    [DHa,DHb,DHal,DHth] = a_paramDH();

    theta1=theta(1) + DHth(1);
    theta2=theta(2) + DHth(2);
    theta3=theta(3) + DHth(3);
    
    D1=DHb(1);
    D2=DHa(2);
    D3=DHa(3);
    e = DHb(3);
    
    J = [-sin(theta1) * (cos(theta2) * D3 * cos(theta3) - sin(theta2) * D3 * sin(theta3) + D2 * cos(theta2)) + cos(theta1) * e cos(theta1) * (-sin(theta2) * D3 * cos(theta3) - cos(theta2) * D3 * sin(theta3) - D2 * sin(theta2)) cos(theta1) * (-cos(theta2) * D3 * sin(theta3) - sin(theta2) * D3 * cos(theta3)); cos(theta1) * (cos(theta2) * D3 * cos(theta3) - sin(theta2) * D3 * sin(theta3) + D2 * cos(theta2)) + sin(theta1) * e sin(theta1) * (-sin(theta2) * D3 * cos(theta3) - cos(theta2) * D3 * sin(theta3) - D2 * sin(theta2)) sin(theta1) * (-cos(theta2) * D3 * sin(theta3) - sin(theta2) * D3 * cos(theta3)); 0 cos(theta2) * D3 * cos(theta3) - sin(theta2) * D3 * sin(theta3) + D2 * cos(theta2) -sin(theta2) * D3 * sin(theta3) + cos(theta2) * D3 * cos(theta3)];

    Jinv=pinv(J);