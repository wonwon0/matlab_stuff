function h = circle2D(points,r,color, thickness)
    %x and y are the coordinates of the center of the circle
    %r is the radius of the circle
    %0.01 is the angle step, bigger values will draw the circle faster but
    %you might notice imperfections (not very smooth)

    x = points(1);
    y= points(2);
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    h = plot(x+xp,y+yp, 'LineWidth',thickness, 'color', color);

end

