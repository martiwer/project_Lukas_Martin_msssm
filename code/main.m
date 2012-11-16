%clean up
clear
clc
format long

%States of the Cars [x, v, acc, v0, T, lane, type(1:car; 2:truck)]
v0_car0_1 = (120+(randn*5))/3.6;    %Desired velocity: 120 km/h with normal distribution with variance 5
v0_car0_2 = (120+(randn*5))/3.6;
T_car0_1 = 2+(randn*0.4);           %Safe time headway: 2 s with normal distribution with variance 0.4
T_car0_2 = 2+(randn*0.4);
state_1 = [0 v0_car0_1 0 v0_car0_1 T_car0_1 0 1];  %State of lane 1
state_2 = [0 v0_car0_2 0 v0_car0_2 T_car0_2 0 1];  %State of lane 2


%Indexes of the state matrix
ix = 1;         %Position
iv = 2;         %Velocity
iacc = 3;       %Acceleration
iv0 = 4;        %Desired velocity
iT = 5;         %Safe time headway
ilane = 6;      %Lane
itype = 7;      %Type 1:car; 2:truck


%Parameters
dt = 0.1;          %Time Step [s]
simend = 3000;     %Simulation time [s]
time = 0;          %Elapsed time [s]
nct = 3;           %New Car Time [s]
pitnc_1 = 0;         %Point in time new car [s] lane1    
pitnc_2 = 0;         %Point in time new car [s] lane2
l_highway = 500;  %Length Highway [m]
n_1=2;             %First updated car lane 1
n_2=2;             %First updated car lane 2


%Intelligent Driver Model parameters
%Car
delta = 4;        %describes acceleration behavior
a_c = 0.6;        %maximal acceleration [m^2/s]
b_c = 0.9;        %maximal decceleration [m^2/S]
l_c = 5;          %vehicle length [m]
s0 = 2;           %jam distance [m]

%Truck
a_t = 0.2;        %maximal acceleration [m^2/s]
b_t = 0.4;        %maximal decceleration [m^2/S]
l_t = 16;         %vehicle length [m]

%Time loop
for time = 0:dt:simend    
    
    %Update accelerations of the vehicles from lane 1
    if(size(state_1,1) > 1)
 
        if(state_1(n_1,ix)>l_highway)   %Check end of highway
            n_1=n_1+1;
        end
        
        for vehicle = n_1:size(state_1,1)
            
            switch state_1(vehicle, itype)  %Vehicle type?
                case 1          %Car
                    a = a_c;
                    b = b_c;
                    l = l_c;
                case 2          %Truck
                    a = a_t;
                    b = b_t;
                    l = l_t;
            end
                       
            x = state_1(vehicle,ix);       %Position of this vehicle
            xm1 = state_1(vehicle-1,ix);   %Position of vehicle in front
            v = state_1(vehicle,iv);       %Velocity of this vehicle
            vm1 = state_1(vehicle-1,iv);   %Velocity of vehicle in front
            v0 = state_1(vehicle,iv0);     %Desired velocity of this vehicle
            T = state_1(vehicle,iT);       %Safe time headway of this vehicle
            
            ds = xm1 - x - l;       %gap to the vehicle in front
            dv = v - vm1;           %velocity difference to the vehicle in front
            ss = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0);       %effective desired distance
            
            state_1(vehicle,iacc) = a*(1 - (v/v0)^delta - (ss/ds)^2);    %new acceleration
            
        end
    end
    
    %Update accelerations of the vehicles from lane 2
    if(size(state_2,1) > 1)
 
        if(state_2(n_2,ix)>l_highway)   %Check end of highway
            n_2=n_2+1;
        end
        
        for vehicle = n_2:size(state_2,1)
            
            switch state_2(vehicle, itype)  %Vehicle type?
                case 1          %Car
                    a = a_c;
                    b = b_c;
                    l = l_c;
                case 2          %Truck
                    a = a_t;
                    b = b_t;
                    l = l_t;
            end
                       
            x = state_2(vehicle,ix);       %Position of this vehicle
            xm1 = state_2(vehicle-1,ix);   %Position of vehicle in front
            v = state_2(vehicle,iv);       %Velocity of this vehicle
            vm1 = state_2(vehicle-1,iv);   %Velocity of vehicle in front
            v0 = state_2(vehicle,iv0);     %Desired velocity of this vehicle
            T = state_2(vehicle,iT);       %Safe time headway of this vehicle
            
            ds = xm1 - x - l;       %gap to the vehicle in front
            dv = v - vm1;           %velocity difference to the vehicle in front
            ss = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0);       %effective desired distance
            
            state_2(vehicle,iacc) = a*(1 - (v/v0)^delta - (ss/ds)^2);    %new acceleration
            
        end
    end
    
    %Update velocities and positions of the cars from lane 1
    state_1(n_1-1,ix) = state_1(n_1-1,ix) + state_1(n_1-1,iv)*dt;
    for vehicle=n_1:size(state_1,1)
        
        x = state_1(vehicle,ix);       %Position of this vehicle
        v = state_1(vehicle,iv);       %Velocity of this vehicle
        acc = state_1(vehicle,iacc);   %Acceleration of this vehicle
        
        state_1(vehicle,iv) = v + acc*dt;    %Update velocity
        if(state_1(vehicle,iv)<0) 
            state_1(vehicle,iv) = 0;
        end
        state_1(vehicle,ix) = x + v*dt;      %Update position
        
    end
    
    %Update velocities and positions of the cars from lane 2
    state_2(n_2-1,ix) = state_2(n_2-1,ix) + state_2(n_2-1,iv)*dt;
    for vehicle=n_2:size(state_2,1)
        
        x = state_2(vehicle,ix);       %Position of this vehicle
        v = state_2(vehicle,iv);       %Velocity of this vehicle
        acc = state_2(vehicle,iacc);   %Acceleration of this vehicle
        
        state_2(vehicle,iv) = v + acc*dt;    %Update velocity
        if(state_2(vehicle,iv)<0) 
            state_2(vehicle,iv) = 0;
        end
        state_2(vehicle,ix) = x + v*dt;      %Update position
        
    end
    
    %Add new vehicle Lane 1
    if(state_1(size(state_1,1),ix)>70 && (time-pitnc_1)>nct)
        
        %Determine type of new vehicle
        if(rand>0.2)
            type=1;
        else
            type=2;
        end
        
        switch type
            case 1
                v0_new = (120+(randn*5))/3.6;       %Desired velocity: 120 km/h with normal distribution with variance 5
                T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
                
                new_vehicle = [0 v0_new 0 v0_new T_new 0 1];       %Create new vehicle [x, v, acc, v0, T, lane, type(1:car; 2:truck)]
                
                state_1 =[state_1; new_vehicle];    %Add new vehicle
            case 2
                v0_new = (80+(randn*4))/3.6;       %Desired velocity: 80 km/h with normal distribution with variance 4
                T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
                
                new_vehicle = [0 v0_new 0 v0_new T_new 0 2];       %Create new vehicle [x, v, acc, v0, T, lane, type(1:car; 2:truck)]
                
                state_1 =[state_1; new_vehicle];    %Add new vehicle
        end
                
        pitnc_1 = time;
        
    end
    
    %Add new vehicle Lane 2
    if(state_2(size(state_2,1),ix)>70 && (time-pitnc_2)>nct)
        
        %Determine type of new vehicle
        if(rand>0.2)
            type=1;
        else
            type=2;
        end
        
        switch type
            case 1
                v0_new = (120+(randn*5))/3.6;       %Desired velocity: 120 km/h with normal distribution with variance 5
                T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
                
                new_vehicle = [0 v0_new 0 v0_new T_new 0 1];       %Create new vehicle [x, v, acc, v0, T, lane, type(1:car; 2:truck)]
                
                state_2 =[state_2; new_vehicle];    %Add new vehicle
            case 2
                v0_new = (80+(randn*4))/3.6;       %Desired velocity: 80 km/h with normal distribution with variance 4
                T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
                
                new_vehicle = [0 v0_new 0 v0_new T_new 0 2];       %Create new vehicle [x, v, acc, v0, T, lane, type(1:car; 2:truck)]
                
                state_2 =[state_2; new_vehicle];    %Add new vehicle
        end
                
        pitnc_2 = time;
        
    end
    
%     if (time==15)
%         state(2,iv0) = 0.1;
%     end
    
     %plot street
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
     % plot cars
     for i = 1:size(state_1)
         draw_car(state_1(i,ix), y_dist/2, 10, 4, state_1(i,itype));
     end
     for i = 1:size(state_2)
         draw_car(state_2(i,ix), -y_dist/2, 10, 4, state_2(i,itype));
     end
     % plot time
        text(4/5*l_highway, 1.15*y_dist, ['time= ' num2str(time)])
    pause(0.05)
    
end
