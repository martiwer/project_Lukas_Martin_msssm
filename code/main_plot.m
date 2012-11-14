%clean up
clear
clc
format long

%States of the Cars [v0, s0, x, v, a]
v0_car0 = (120+(-20+rand(1)*40))/3.6;
s0_car0 = 60+(-20+rand(1)*40);
state = [v0_car0 s0_car0 0 v0_car0 0];

%Variables
dt = 0.05;          %Time Step [s]
simend = 20.05;     %Simulation time [s]
time = 0;           %Elapsed time [s]
nct = 5;            %New Car Time [s]
l = 10000;          % Length of highway

%Time loop
for time = 0:dt:simend
    
    
    %Update accelerations of the cars
    for car = 1:size(state,1)
        
        a = 0;
        
    end
    
    %Update velocities and positions of the cars
    for car=1:size(state,1)
        
        state(car,4) = state(car,4) + a*dt;
        state(car,3) = state(car,3) + state(car,4)*dt
        
    end
    
    
    if((time > 0) && (mod(time,nct) == 0))
        newcar = [(120+(-20+rand(1)*40))/3.6 60+(-20+rand(1)*40), 0, (120+(-20+rand(1)*40))/3.6, 0];
        state = [state; newcar];
    end
    
    % plot cars
    clf;
        y_dist=1;
        plot(0:l, 0*(0:l), 'Color', [.75 .75 .75], 'LineWidth', 400)
        hold on;
        plot([0,l] ,[0,0],'--w','LineWidth', 1)
        hold on;
        plot([0,l] ,[-y_dist,-y_dist],'k','LineWidth', 2)
        hold on;
        plot([0,l] ,[y_dist,y_dist],'k','LineWidth', 2)
        xlim([0 l])
        ylim([-1.5*y_dist 1.5*y_dist])
        
        c_color=1;
     for i = 1:size(state)
         draw_car(state(i,3), y_dist/2, 10, 6, c_color);
     end
    pause(0.05)
end
