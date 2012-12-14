function draw_car(x0, y0, w, h, t, c)

% This function is drawing a car
% INPUT:
% x0, y0: Central point of the car
% w: Width of the car
% h: Height of the car
% Hold the graphics

hold on

% Define the coordinates for the car chassi
chassi_x = [w/2 w/2 2*w/6 w/6 -2*w/6 -w/2 -w/2];
chassi_y = [0 h/2 h/2 h h h/2 0]*0.01;
chassi_l_x = [3/2*w 3/2*w -w/2 -w/2];
chassi_l_y = [0 3/2*h 3/2*h 0]*0.01;
chassi_cl_x = [3/2*w 3/2*w 2*w 2*w];
chassi_cl_y = [0 5/4*h 5/4*h 0]*0.01;

% Define the coordinates for the wheels
angles = 0:0.1:(2*pi);
r = sqrt(w*w + h*h)/1000;
wheel_x = r*cos(angles)*100;
wheel_y = r*sin(angles);

% Draw the car!
if(t==1)
    if(c==1)
        patch(x0+chassi_x, y0+chassi_y, 'r')
        patch(x0-w/4+wheel_x, y0+wheel_y, 'k')
        patch(x0+w/4+wheel_x, y0+wheel_y, 'k')
    end
    if(c==2)
        patch(x0+chassi_x, y0+chassi_y, 'g')
        patch(x0-w/4+wheel_x, y0+wheel_y, 'k')
        patch(x0+w/4+wheel_x, y0+wheel_y, 'k')
    end
end
%if(c==0)
%patch(x0+chassi_x, y0+chassi_y, 'y')
%end
if(t==2)
patch(x0+chassi_l_x, y0+chassi_l_y, 'b')
patch(x0+chassi_cl_x, y0+chassi_cl_y, [1,0,1])
patch(x0-w/4+wheel_x, y0+wheel_y, 'k')
patch(x0+wheel_x, y0+wheel_y, 'k')
patch(x0+7/4*w+wheel_x, y0+wheel_y, 'k')
end
