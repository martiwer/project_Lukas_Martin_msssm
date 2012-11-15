%clean up
clear
clc
format long

%States of the Cars [v0, s0, T, x, v, acc]
v0_car0 = (120+(-20+rand(1)*40))/3.6;   %Desired velocity: 120 km/h with tolerance +/- 20
s0_car0 = 2+(-1+rand(1)*2);             %Jam distance: 2 m with tolerance +/- 1
T_car0 = 2+(-0.5+rand(1)*1);            %Safe time headway: 2 s with tolerance +/- 0.5
state = [v0_car0 s0_car0 T_car0 0 v0_car0 0];

%Parameter
dt = 0.4;           %Time Step [s]
simend = 3000;       %Simulation time [s]
time = 0;           %Elapsed time [s]
nct = 10;            %New Car Time [s]
pitnc = 0;          %Point in time new car [s]      
l_highway = 10000;  %Length Highway [m]
n=2;                %First updated car

%Intelligent Driver Model parameters
delta = 4;      %describes acceleration behavior
a = 0.6;        %acceleration [m^2/s]
b = 0.9;        %decceleration [m^2/S]
l = 5;          %vehicle length [m]
 



%Time loop
for time = 0:dt:simend    
    
    %Update accelerations of the cars
    if(size(state,1) > 1)
 
        if (state(n,4)>l_highway)   %Check end of highway
            n=n+1;
        end
        
        for car = n:size(state,1)
            
            x = state(car,4);       %Position of this car
            xm1 = state(car-1,4);   %Position of car in front
            v = state(car,5);       %Velocity of this car
            vm1 = state(car-1,5);   %Velocity of car in front
            v0 = state(car,1);      %Desired velocity of this car
            s0 = state(car,2);      %Jam distance of this car
            T = state(car,3);       %Safe time headway of this car
            
            ds = xm1 - x - l;       %gap to the car in front
            dv = v - vm1;           %velocity difference to the car in front
            ss = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0);       %effective desired distance
            
            state(car,6) = a*(1 - (v/v0)^delta - (ss/ds)^2);    %new acceleration
            
        end
    end
    
    %Update velocities and positions of the cars
    for car=1:size(state,1)
        
        x = state(car,4);       %Position of this car
        v = state(car,5);       %Velocity of this car
        acc = state(car,6);       %Acceleration of this car
        
        state(car,5) = v + acc*dt;    %Update velocity
        if(state(car,5)<0) 
            state(car,5) = 0;
        end
        state(car,4) = x + v*dt;    %Update position
        
    end
    
    %Add new Car
    if(state(size(state,1),4)>70 && (time-pitnc)>nct)
        
        v0_car = (120+(-20+rand(1)*40))/3.6;   %Desired velocity: 120 km/h with tolerance +/- 20
        s0_car = 2+(-1+rand(1)*2);             %Jam distance: 2 m with tolerance +/- 1
        T_car = 2+(-0.5+rand(1)*1);            %Safe time headway: 2 s with tolerance +/- 0.5
        
        newcar = [v0_car s0_car T_car 0 v0_car 0];
        state = [state; newcar];
        pitnc = time;
    end
    
    if (time==15)
        state(2,5) = 0;
    end
    
    % plot cars
    clf;
        y_dist=0.5;
        plot(0:l_highway, 0*(0:l_highway), 'Color', [.75 .75 .75], 'LineWidth', 600)
        hold on;
        plot([0,l_highway] ,[0,0],'--w','LineWidth', 1)
        hold on;
        plot([0,l_highway] ,[-y_dist,-y_dist],'k','LineWidth', 2)
        hold on;
        plot([0,l_highway] ,[y_dist,y_dist],'k','LineWidth', 2)
        xlim([0 l_highway])
        ylim([-1.5*y_dist 1.5*y_dist])
        xlabel('Position [m]')
 
        c_color=1;
     for i = 1:size(state)
         draw_car(state(i,4), y_dist/2, 10, 4, c_color);
         text(4/5*l_highway, 1.15*y_dist, ['time= ' num2str(time)])
     end
     
    pause(0.00005)
    
    
end
