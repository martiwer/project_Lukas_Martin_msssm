function simulate_main(simtime, l_highway, disturbance, speed_limit, dir, withplot)
%simulate_main(simtime, l_highway, disturbance, speed_limit, dir, withplot)
%Simulates the traffic flow for the duration of simtime
%disturbance(0:without; 1:lane2; 2:both)
%dir: direction

%clean up
close all;
clc;
format long

%Indexes of the state matrix
ix = 1;             %Position
iv = 2;             %Velocity
iacc = 3;           %Acceleration
iv0 = 4;            %Desired velocity
iT = 5;             %Safe time headway
icp = 6;            %Changed position
itype = 7;          %Type 1:car; 2:truck
iol = 8;            %Original lane 
idili = 9;          %Disturbance and speed limit (0:nothing, 1: only speed limit, 2:only disturbed, 3:both)
itime = 10;         %Car time elapsed on highway

%Parameters
dt = 0.1;          %Time Step [s] (needs to be 0.1!!)
simend = simtime;  %Simulation time [s]
nct_1 = 2;         %New Car Time lane 1 [s]
nct_2 = 5;         %New Car Time lane 2 [s]
T_d = 1;           %Dead time (has to be a multiple of 0.1)
dist_factor = 0.6; %Disturbance factor
pitnc_1 = 0;       %Point in time new car [s] lane1    
pitnc_2 = 0;       %Point in time new car [s] lane2
l_highway;         %Length Highway [m]
n_1=2;             %First updated car lane 1
n_2=2;             %First updated car lane 2

%Initialize Variables
time = 0;          %Elapsed time [s]
ec1 = 0;           %Number of entering cars lane 1
lc1 = 0;           %Number of leaving cars lane 1
ec2 = 0;           %Number of entering cars lane 2
lc2 = 0;           %Number of leaving cars lane 2
type = 1;          %Initialize type
av_time_c = 0;     %Average transit time car
av_time_t = 0;     %Average transit time truck
lcc = 0;           %Leaving car counter
ltc = 0;           %Leaving truck counter
crash = 0;         %Crash Detection

%States of the Cars [x, v, acc, v0, T, changedposition, type(1:car; 2:truck), original lane, disturbance]
v0_car0 = 120/3.6;    
T_car0 = 2;           
state_1 = [l_highway+100 v0_car0 0 v0_car0 T_car0 0 1 1 0 0];  %State of lane 1
state_2 = [l_highway+100 v0_car0 0 v0_car0 T_car0 0 1 2 0 0];  %State of lane 2

%Intelligent Driver Model parameters
%Car
delta = 5;        %describes acceleration behavior (velocity difference)
epsilon = 3;      %describes acceleration behavior (gap)
a_c = 0.6;        %maximal acceleration [m/s^2]
b_c = 0.9;        %maximal decceleration [m/s^2]
l_c = 5;          %vehicle length [m]
s0 = 2;           %jam distance [m]

%Truck
a_t = 0.2;        %maximal acceleration [m/s^2]
b_t = 0.4;        %maximal decceleration [m/s^2]
l_t = 16;         %vehicle length [m]

%Save Parameters
save([dir '/Parameters'])

%Time loop
for time = 0:dt:simend
    
    %Increase elapsed time
    time_1 = ones(size(state_1,1),1)*dt;
    time_2 = ones(size(state_2,1),1)*dt;
    state_1(:,itime) = state_1(:,itime) + time_1;
    state_2(:,itime) = state_2(:,itime) + time_2;
    
    %Change vehicles from lane 1 to lane 2
    if(mod(time, 0.2) == 0)        
        %Tag changing cars from lane 1 to lane 2
        state_1(:,icp) = zeros(size(state_1,1),1);
        if(size(state_1,1) > 2)
            %Delete car which has reached l_highway
            if(state_1(n_1,ix)>l_highway)
                av_time_c = av_time_c + state_1(1,itime);
                lcc = lcc + 1;
                state_1(1,:) = [];
                lc1 = lc1 + 1;
            end
            for vehicle_1 = n_1:size(state_1,1)-1

                %Determine vehicle on lane 2 ahead/ behind this vehicle
                for vehicle_2 = n_2:size(state_2,1) 

                    if(state_2(vehicle_2, ix) < state_1(vehicle_1, ix))
                        vehicle_a = vehicle_2-1;   %Vehicle ahead
                        vehicle_b = vehicle_2;     %Vehicle behind

                        %Check lane change situation
                        v_1 = state_2(vehicle_a, iv);
                        v = state_1(vehicle_1, iv);
                        dv = v - v_1;
                        T = state_1(vehicle_1, iT);
                        a = a_c;
                        b = b_c;
                        l = l_c;
                        s_a = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0) + l;           %Desired gap to vehicle ahead
                        
                        v_1 = state_1(vehicle_1, iv);
                        v = state_2(vehicle_b, iv);
                        dv = v - v_1;
                        T = state_2(vehicle_b, iT);
                        switch state_2(vehicle_b, itype)
                            case 1          %Car
                                a = a_c;
                                b = b_c;
                                l = l_c;
                            case 2          %Truck
                                a = a_t;
                                b = b_t;
                                l = l_t;
                        end
                        s_b = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0) + l;           %Desired gap to vehicle behind
                        
                        c1 = state_2(vehicle_a, ix)-state_1(vehicle_1, ix) > 300;    %Gap to vehicle ahead_2 > 300
                        c2 = state_1(vehicle_1, ix)-state_2(vehicle_b, ix) > s_b;    %Gap to vehicle behind_2 big enough
                        c3 = state_1(vehicle_1, iv0) > state_1(vehicle_1-1, iv);     %v0 > v of vehicle ahead_1
                        c4 = state_2(vehicle_a, iv) > state_1(vehicle_1-1, iv);      %v of vehicle ahead_2 > v of vehicle ahead_1
                        c5 = state_2(vehicle_a, ix)-state_1(vehicle_1, ix) > s_a;    %Gap to vehicle ahead_2 big enough
                        c6 = state_2(vehicle_a, iv) < state_1(vehicle_1, iv0);       %v of vehicle ahead_2 < v0
                        c7 = state_1(vehicle_1-1, ix)-state_1(vehicle_1, ix) > 200;  %Gap to vehicle ahead_1 > 200

                        if((c1 && c2) || (c3 && c4 && c5 && c2 && not(c6 && c7)))
                            state_1(vehicle_1, icp) = vehicle_b;    %Set changed position
                        end

                    end

                end

            end

        end

        %Change the vehicles lane 1 to 2
        ccc1 = 0; % Set changing car counter lane 1
        for vehicle_1 = n_1:size(state_1,1)-1

            if (state_1(vehicle_1, icp)>0)
                pos = state_1(vehicle_1, icp) + ccc1;
                state_2_temp_1 = state_2(1:pos-1,:);
                state_2_temp_2 = state_2(pos:size(state_2,1),:);
                state_2 = [state_2_temp_1; state_1(vehicle_1,:); state_2_temp_2];
                ccc1 = ccc1 + 1;
                state_2(pos, icp) = -1;
            end

        end

        %Delete changed cars lane 1
        for i=1:ccc1

            for vehicle_1=n_1:size(state_1,1)-1

                if (state_1(vehicle_1, icp)>0)
                state_1(vehicle_1,:)=[];
                break
                end
            end

        end
    end
   
    %Change vehicles from lane 2 to lane 1
    if(mod(time, 0.2) == 0.1)        
        %Tag changing cars from lane 2 to lane 1
        state_2(:,icp) = zeros(size(state_2,1),1);
        if(size(state_2,1) > 2)
            %Delete car which has reached l_highway
            if(state_2(n_2,ix)>l_highway)
                if(state_2(1,itype) == 1)
                    av_time_c = av_time_c + state_2(1,itime);
                    lcc = lcc + 1;
                else
                    av_time_t = av_time_t + state_2(1,itime);
                    ltc = ltc + 1;
                end
                state_2(1,:) = [];
                lc2 = lc2 + 1;
            end
            for vehicle_2 = n_2:size(state_2,1)-1

                if(state_2(vehicle_2, itype)==1)
                    %Determine vehicle on lane 2 ahead/ behind this vehicle
                    for vehicle_1 = n_1:size(state_1,1) 

                        if(state_1(vehicle_1, ix) < state_2(vehicle_2, ix))
                            vehicle_a = vehicle_1-1;   %Vehicle ahead
                            vehicle_b = vehicle_1;     %Vehicle behind

                            %Check lane change situation
                            v_1 = state_1(vehicle_a, iv);
                            v = state_2(vehicle_2, iv);
                            dv = v - v_1;
                            T = state_2(vehicle_2, iT);
                            switch state_2(vehicle_2, itype)
                                case 1          %Car
                                    a = a_c;
                                    b = b_c;
                                    l = l_c;
                                case 2          %Truck
                                    a = a_t;
                                    b = b_t;
                                    l = l_t;
                            end
                            s_a = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0) + l;           %Desired gap to vehicle ahead
                            
                            v_1 = state_2(vehicle_2, iv);
                            v = state_1(vehicle_b, iv);
                            dv = v - v_1;
                            T = state_1(vehicle_b, iT);
                            a = a_c;
                            b = b_c;
                            l = l_c;
                            s_b = s0 + max(v*T + (v*dv)/(2*sqrt(a*b)), 0) + l;           %Desired gap to vehicle behind
                            
                            c1 = state_1(vehicle_a, ix)-state_2(vehicle_2, ix) > 300;    %Gap to vehicle ahead_1 > 300
                            c2 = state_2(vehicle_2-1, ix)-state_2(vehicle_2, ix) < 200;  %Gap to vehicle ahead_2 < 200
                            c3 = state_2(vehicle_2-1, iv) < state_2(vehicle_2, iv0);     %v of vehicle ahead_2 < v0  
                            c4 = state_2(vehicle_2, ix)-state_1(vehicle_b, ix) > s_b;    %Gap to to vehicle behind_1 big enough
                            c5 = state_1(vehicle_a, iv) > state_2(vehicle_2-1, iv);      %v of vehicle ahead_1 > v of vehicle ahead_2
                            c6 = state_1(vehicle_a, ix)-state_2(vehicle_2, ix) > s_a;    %Gap to vehicle ahead_1 big enough


                            if((c1 && c2 && c3 && c4) || (c2 && c3 && c5 && c6 && c4))
                                state_2(vehicle_2, icp) = vehicle_b;    %Set changed position
                            end

                        end

                    end
                end
            end

        end

        %Change the vehicles lane 2 to 1
        ccc2 = 0;   % Set changing car counter lane 2
        for vehicle_2 = n_2:size(state_2,1)-1

            if (state_2(vehicle_2, icp)>0)
                pos = state_2(vehicle_2, icp) + ccc2;
                state_1_temp_1 = state_1(1:pos-1,:);
                state_1_temp_2 = state_1(pos:size(state_1,1),:);
                state_1 = [state_1_temp_1; state_2(vehicle_2,:); state_1_temp_2];
                ccc2 = ccc2 + 1;
                state_1(pos, icp) = -1;
            end

        end

        % Delete changed cars lane 2
        for i=1:ccc2

            for vehicle_2=n_2:size(state_2,1)-1
                if (state_2(vehicle_2, icp)>0)
                state_2(vehicle_2,:)=[];
                break
                end
            end

        end
    end
    
    %Update accelerations of the vehicles from lane 1
    if(mod(time,T_d) == 0)
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
          
            state_1(vehicle,iacc) = a*(1 - (v/v0)^delta - (ss/ds)^epsilon);    %new acceleration

        end
        
    end
    
    %Update accelerations of the vehicles from lane 2
    if(mod(time,T_d) == 0)
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
            
            state_2(vehicle,iacc) = a*(1 - (v/v0)^delta - (ss/ds)^epsilon);    %new acceleration
            
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
    v0_new = (120+(randn*8))/3.6;       %Desired velocity: 120 km/h with normal distribution with variance 8
    T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
    dv = v0_new - state_1(size(state_1,1), iv);
    ss = s0 + max(v0_new*T_new + (v0_new*dv)/(2*sqrt(a_c*b_c)), 0);
    
    if(state_1(size(state_1,1),ix)>ss && (time-pitnc_1)>nct_1)
            new_vehicle = [0 v0_new 0 v0_new T_new 0 1 1 0 0];       %Create new vehicle [x, v, acc, v0, T, changedposition, type(1:car; 2:truck), original lane, disturbance]

            state_1 = [state_1; new_vehicle];    %Add new vehicle
                
            pitnc_1 = time;
            ec1 = ec1 + 1;
    end
    
    %Add new vehicle Lane 2        
    switch type
        case 1
            v0_new = (120+(randn*8))/3.6;       %Desired velocity: 120 km/h with normal distribution with variance 8
            v_new = (80+(randn*6))/3.6;         %Actual incoming velocity: 80 km/h with normal distribution with variance 6
            T_new = 2+(randn*0.4);              %Safe time headway: 2 s with normal distribution with variance 0.4
            dv = v_new - state_2(size(state_2,1), iv);
            ss = s0 + max(v_new*T_new + (v_new*dv)/(2*sqrt(a_c*b_c)), 0);
            
            if(state_2(size(state_2,1),ix)>ss && (time-pitnc_2)>nct_2) 
                new_vehicle = [0 v_new 0 v0_new T_new 0 1 2 0 0];        %Create new vehicle [x, v, acc, v0, T, changedposition, type(1:car; 2:truck), original lane, disturbance]
                
                state_2 = [state_2; new_vehicle];    %Add new vehicle
                
                pitnc_2 = time;
                ec2 = ec2 + 1;
                
                %Determine type of next new vehicle
                if(rand>0.2)
                        type=1;
                    else
                        type=2;
                end
                
            end
            
        case 2
            v0_new = (80+(randn*4))/3.6;       %Desired velocity: 80 km/h with normal distribution with variance 4
            T_new = 3+(randn*0.4);             %Safe time headway: 2 s with normal distribution with variance 0.4
            dv = v0_new - state_2(size(state_2,1), iv);
            ss = s0 + max(v0_new*T_new + (v0_new*dv)/(2*sqrt(a_t*b_t)), 0);
            
            if(state_2(size(state_2,1),ix)>ss && (time-pitnc_2)>nct_2)
                new_vehicle = [0 v0_new 0 v0_new T_new 0 2 2 0 0];        %Create new vehicle [x, v, acc, v0, T, changedposition, type(1:car; 2:truck), original lane, disturbance]
                
                state_2 = [state_2; new_vehicle];    %Add new vehicle
                
                pitnc_2 = time;
                ec2 = ec2 + 1;
                
                %Determine type of next new vehicle
                if(rand>0.2)
                        type=1;
                    else
                        type=2;
                end
                
            end
            
    end
    
    %Percentage of the progress
    perpro = time/simend*100;
    if(mod(perpro, 1) == 0)
        %clc
        disp(['Progress: ' num2str(perpro) '%'])
    end
    
    %Check of state_1 sort
    for vehicle_1=n_1:size(state_1,1)
        if(state_1(vehicle_1, ix) > state_1(vehicle_1-1, ix) && size(state_1,1) > 1)
            disp('Attention: state_1 not sorted well!!!')
            crash = 1;
        end
    end
    
    %Check of state_2 sort
    for vehicle_2=n_2:size(state_2,1)
        if(state_2(vehicle_2, ix) > state_2(vehicle_2-1, ix) && size(state_2,1) > 1)
            disp('Attention: state_2 not sorted well!!!')
            crash = 1;
        end
    end
    
    %Save states every second
    if (mod(time, 1) == 0)
        %save data\[statefile 'num2str(time)'] time state_1 state_2
        save([dir '/statefile_' num2str(time)], 'time', 'state_1', 'state_2', 'ec1', 'ec2', 'lc1', 'lc2', 'av_time_c', 'av_time_t', 'lcc', 'ltc', 'crash')
    end
    
    %Disturbance and Speed Limit
    pos_dist_start = l_highway * 0.5;         %Starting point of disturbance
    pos_dist_end = pos_dist_start + 1000;     %End point of disturbance
    time_dist_start = simend * 0.2;           %Start time of disturbance
    time_dist_end = time_dist_start + 600;    %End time of disturbance

    pos_limit_start = l_highway * 0.3;        %Starting point of speed limit
    pos_limit_end = pos_dist_end + 1000;      %End point of speed limit
    time_limit_start = simend * 0.15;         %Start time of speed limit
    time_limit_end = time_dist_end + 300;     %End time of speed limit
    
    limit_factor = speed_limit/120;           %Speed limit factor
    dist_factor_l = dist_factor/limit_factor; %Disturbance factor with speed limit
    
    %Speed limit
    if(speed_limit > 0)
        %Lane1
        for vehicle_1 = n_1:size(state_1,1)

            if(time > time_limit_start && time < time_limit_end && state_1(vehicle_1, idili) == 0 && state_1(vehicle_1, ix) > pos_limit_start && state_1(vehicle_1, ix) < pos_limit_end)
                state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * limit_factor;           %Decrease Velocity
                state_1(vehicle_1, idili) = 1;
            end

            if(state_1(vehicle_1, idili) == 1 && (not(time > time_limit_start && time < time_limit_end) || not(state_1(vehicle_1, ix) > pos_limit_start && state_1(vehicle_1, ix) < pos_limit_end)))
                    state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * (1/limit_factor);    %Increase Velocity
                    state_1(vehicle_1, idili) = 0;
            end
        end

        %Lane2
        for vehicle_2 = n_2:size(state_2,1)

            if(time > time_limit_start && time < time_limit_end && state_2(vehicle_2, itype) == 1 && state_2(vehicle_2, idili) == 0 && state_2(vehicle_2, ix) > pos_limit_start && state_2(vehicle_2, ix) < pos_limit_end)
                state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * limit_factor;            %Decrease Velocity
                state_2(vehicle_2, idili) = 1;
            end

            if(state_2(vehicle_2, itype) == 1 && state_2(vehicle_2, idili) == 1 && (not(time > time_limit_start && time < time_limit_end) || not(state_2(vehicle_2, ix) > pos_limit_start && state_2(vehicle_2, ix) < pos_limit_end)))
                    state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * (1/limit_factor);    %Increase Velocity
                    state_2(vehicle_2, idili) = 0;
            end
        end
    end
          
    %Disturbance    
    switch disturbance
        case 1
            %Lane1
            for vehicle_1 = n_1:size(state_1,1)
                
                 if(state_1(vehicle_1, idili) == 2)
                     state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * (1/dist_factor);     %Increase Velocity without speed limit
                     state_1(vehicle_1, idili) = 0;
                 end
                 if(state_1(vehicle_1, idili) == 3)
                     state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * (1/dist_factor_l);     %Increase Velocity with speed limit
                     state_1(vehicle_1, idili) = 1;
                 end
                
            end
            
            %Lane2
            for vehicle_2 = n_2:size(state_2,1)

                if(time > time_dist_start && time < time_dist_end && state_2(vehicle_2, itype) == 1 && state_2(vehicle_2, ix) > pos_dist_start && state_2(vehicle_2, ix) < pos_dist_end)
                    if(state_2(vehicle_2, idili) == 0)
                        state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * dist_factor;          %Decrease Velocity without speed limit
                        state_2(vehicle_2, idili) = 2;
                    end
                    if(state_2(vehicle_2, idili) == 1)
                        state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * dist_factor_l;        %Decrease Velocity with speed limit
                        state_2(vehicle_2, idili) = 3;
                    end
                end
                
                if(state_2(vehicle_2, itype) == 1 && (not(time > time_dist_start && time < time_dist_end) || not(state_2(vehicle_2, ix) > pos_dist_start && state_2(vehicle_2, ix) < pos_dist_end)))
                     if(state_2(vehicle_2, idili) == 2)
                         state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * (1/dist_factor);     %Increase Velocity without speed limit
                         state_2(vehicle_2, idili) = 0;
                     end
                     if(state_2(vehicle_2, idili) == 3)
                         state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * (1/dist_factor_l);     %Increase Velocity with speed limit
                         state_2(vehicle_2, idili) = 1;
                     end
                end
            end    
          
        case 2
            %Lane1
            for vehicle_1 = n_1:size(state_1,1)

                if(time > time_dist_start && time < time_dist_end && state_1(vehicle_1, itype) == 1 && state_1(vehicle_1, ix) > pos_dist_start && state_1(vehicle_1, ix) < pos_dist_end)
                    if(state_1(vehicle_1, idili) == 0)
                        state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * dist_factor;          %Decrease Velocity without speed limit
                        state_1(vehicle_1, idili) = 2;
                    end
                    if(state_1(vehicle_1, idili) == 1)
                        state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * dist_factor_l;        %Decrease Velocity with speed limit
                        state_1(vehicle_1, idili) = 3;
                    end
                end
                
                if(state_1(vehicle_1, itype) == 1 && (not(time > time_dist_start && time < time_dist_end) || not(state_1(vehicle_1, ix) > pos_dist_start && state_1(vehicle_1, ix) < pos_dist_end)))
                     if(state_1(vehicle_1, idili) == 2)
                         state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * (1/dist_factor);     %Increase Velocity without speed limit
                         state_1(vehicle_1, idili) = 0;
                     end
                     if(state_1(vehicle_1, idili) == 3)
                         state_1(vehicle_1, iv0) = state_1(vehicle_1, iv0) * (1/dist_factor_l);     %Increase Velocity with speed limit
                         state_1(vehicle_1, idili) = 1;
                     end
                end
            end
            
            %Lane2
            for vehicle_2 = n_2:size(state_2,1)

                if(time > time_dist_start && time < time_dist_end && state_2(vehicle_2, itype) == 1 && state_2(vehicle_2, ix) > pos_dist_start && state_2(vehicle_2, ix) < pos_dist_end)
                    if(state_2(vehicle_2, idili) == 0)
                        state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * dist_factor;          %Decrease Velocity without speed limit
                        state_2(vehicle_2, idili) = 2;
                    end
                    if(state_2(vehicle_2, idili) == 1)
                        state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * dist_factor_l;        %Decrease Velocity with speed limit
                        state_2(vehicle_2, idili) = 3;
                    end
                end
                
                if(state_2(vehicle_2, itype) == 1 && (not(time > time_dist_start && time < time_dist_end) || not(state_2(vehicle_2, ix) > pos_dist_start && state_2(vehicle_2, ix) < pos_dist_end)))
                     if(state_2(vehicle_2, idili) == 2)
                         state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * (1/dist_factor);     %Increase Velocity without speed limit
                         state_2(vehicle_2, idili) = 0;
                     end
                     if(state_2(vehicle_2, idili) == 3)
                         state_2(vehicle_2, iv0) = state_2(vehicle_2, iv0) * (1/dist_factor_l);     %Increase Velocity with speed limit
                         state_2(vehicle_2, idili) = 1;
                     end
                end
            end
    end
    
    %Plot street
    if(withplot)
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
        
        %Plot cars
        for i = 1:size(state_1)
            
            if (state_1(i,icp)==0)
                draw_car_diff_colors(state_1(i,ix), y_dist/2, 10, 4, state_1(i,itype), state_1(i,iol));
            else
                draw_car_diff_colors(state_1(i,ix), 0, 10, 4, state_1(i,itype), state_1(i,iol));
            end
            
        end
        for i = 1:size(state_2)
            
            if (state_2(i,icp)==0)
                draw_car_diff_colors(state_2(i,ix), -y_dist/2, 10, 4, state_2(i,itype), state_2(i,iol));
            else
                draw_car_diff_colors(state_2(i,ix), 0, 10, 4, state_2(i,itype), state_2(i,iol));
            end
            
        end
        
        %Plot time
        text(4/5*l_highway, 1.15*y_dist, ['time= ' num2str(time)])
        pause(0.05)
    end
     
end

end