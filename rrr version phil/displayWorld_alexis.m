function displayWorld(xPGD)
persistent h t1 t2 t3 t4 t5
if isempty(h)
    % Éléments permanent du graph 
    
    h = hgload('workspace_00136.fig');
    hold on
    a = get_ancrage();
    
    scatter3(a(1,:),a(2,:),a(3,:),'b')
    axis equal
    
end
try
    delete(t1)
    delete(t2)
    delete(t3)
    delete(t4)
    delete(t5)
end

% Éléments qui doivent être updatés à chaque fois
[xq,Q] = PGD2QUAT(xPGD);
x = Q*[0 0 0]' + xq(1:3);
xl = Q*[0.2 0 0]' + xq(1:3);
p2 = xPGD(4:6); %Q*[0 0 0.3]' + xq(1:3);
p1 = xPGD(1:3); %Q*[0 0 0.3+get_p2p1dist()]' + xq(1:3);
p3 = get_p3_from_p2p1(p1,p2);
t1 = scatter3(x(1),x(2),x(3),'r');
t2 = plot3([x(1) xl(1)],[x(2) xl(2)],[x(3) xl(3)],'r');
t3 = scatter3(p3(1),p3(2),p3(3),'g');
t4 = scatter3(p2(1),p2(2),p2(3),'g');
t5 = scatter3(p1(1),p1(2),p1(3),'g');
drawnow