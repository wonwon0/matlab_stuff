function [ dir, rot ] = read_joystick_inputs( my_joystick, comp_windows)
    
    if comp_windows
        %(unitaire)               v_x                                               v_y                                              v_z
        a = [(button(my_joystick,3) - button(my_joystick,1)), (button(my_joystick,4) - button(my_joystick,2)), (button(my_joystick,7) - button(my_joystick,8))];

        %(unitaire)     w_x                  w_y                 w_z
        rot = [axis(my_joystick,2), axis(my_joystick,1), axis(my_joystick,3)];
        rot(abs(rot)<0.1)=0;
        if norm(rot) > 0
            rot = rot/norm(rot)/300;
        end

        if (a(1)==0 && a(2)==0 && a(3)==0)
            dir=[0 0 0];
        elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
            dir=[double(a(2)),double(a(1)),-double(a(3))];
        else
            dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
        end
    else
        %(unitaire)     v_x                 v_y                    v_z
        a = [axis(my_joystick,5), axis(my_joystick,6), (button(my_joystick,7) - button(my_joystick,8))];

        %(unitaire)     w_x                  w_y                 w_z
        rot = [axis(my_joystick,2), axis(my_joystick,1), axis(my_joystick,3)];
        if norm(rot) > 0
            rot = rot/norm(rot)/300;
        end

        if (a(1)==0 && a(2)==0 && a(3)==0)
            dir=[0 0 0];
        elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
            dir=[double(a(2)),double(a(1)),-double(a(3))];
        else
            dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
        end
    end


    
end

