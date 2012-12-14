function plot_graphics(simtime, l_highway_start, l_highway_end, withDensity, withVelocity, withVehicleCounter)
%plot_graphics(simtime, l_highway_start, l_highway_end, withDensity, withVelocity, withVehicleCounter)
%plots the traffic density (vehicles/km), the average speed and the outgoing 
%traffic flow over a section of highway from l_highway_start to l_highway_end

%Indexes of the state matrix
ix = 1;         %Position
iv = 2;         %Velocity
iacc = 3;       %Acceleration
iv0 = 4;        %Desired velocity
iT = 5;         %Safe time headway
icp = 6;        %Changed position
itype = 7;      %Type 1:car; 2:truck
iol = 8;        %Original lane

%Define Parameters
l_highway = l_highway_end - l_highway_start;
N = 100;                            %space resolution
x = linspace(l_highway_start,l_highway_end,5*N+1);  %evaluating points
time_vec = linspace(0,simtime,5*N+1);               %time vector
time_vec2 = linspace(0,simtime,N/2+1);              %time vector
Mat_density1 = zeros(5*N+1,5*N+1);  %Spatiotemporal density Matrix lane 1
Mat_density2 = zeros(5*N+1,5*N+1);  %Spatiotemporal density Matrix lane 2
Mat_velocity1 = zeros(5*N+1,5*N+1); %Spatiotemporal velocity Matrix lane 1
Mat_velocity2 = zeros(5*N+1,5*N+1); %Spatiotemporal velocity Matrix lane 2
Mat_ec = zeros(N/2+1,2);            %Entering cars
Mat_lc = zeros(N/2+1,2);            %Leaving cars
ec1_temp = 0;
ec2_temp = 0;
lc1_temp = 0;
lc2_temp = 0;

    %Calculate density
    for timestep = 0:1:5*N
        
        % load statefile
        time = round(time_vec(timestep+1));
        load(['data/statefile_' num2str(time)])
        if(withDensity)
            for place = 1:(5*N+1)
                counter = 0;
                for vehicle_1 = 1:size(state_1,1)
                    
                    if(state_1(vehicle_1,ix) < x(place) - 500)
                        break
                    end
                    if(state_1(vehicle_1,ix) < x(place) + 500)
                        counter = counter + 1;
                    end    
                    
                end
                Mat_density1(timestep+1,place) = counter;
                counter = 0;
                for vehicle_2 = 1:size(state_2,1)
                    
                    if(state_2(vehicle_2,ix) < x(place) - 500)
                        break
                    end
                    if(state_2(vehicle_2,ix) < x(place) + 500)
                        counter = counter + 1;
                    end 
                    
                end
                Mat_density2(timestep+1,place) = counter;
            end
        end
        
        if(withVelocity)
            for place = 1:(5*N+1)
                
                counter = 0;
                av_v = 0;
                for vehicle_1 = 1:size(state_1,1)
                    
                    if(state_1(vehicle_1,ix) < x(place) - 100)
                            break
                    end
                    if(state_1(vehicle_1,ix) < x(place) + 100)
                            counter = counter + 1;
                            av_v = av_v + state_1(vehicle_1,iv);
                    end
                    
                end
                if(counter)
                    Mat_velocity1(timestep+1,place) = av_v/counter;
                else
                    Mat_velocity1(timestep+1,place) = 0;
                end
                counter = 0;
                av_v = 0;
                for vehicle_2 = 1:size(state_2,1)
                    
                    if(state_2(vehicle_2,ix) < x(place) - 100)
                            break
                    end
                    if(state_2(vehicle_2,ix) < x(place) + 100)
                            counter = counter + 1;
                            av_v = av_v + state_2(vehicle_2,iv);
                    end
                    
                end
                if(counter)
                    Mat_velocity2(timestep+1,place) = av_v/counter;
                else
                    Mat_velocity2(timestep+1,place) = 0;
                end
            end
        end
        
        if(withVehicleCounter)
            if(mod(timestep,10) == 0)
            if(time)
                %Entering cars
                Mat_ec(timestep/10+1,1) = ec1 - ec1_temp;
                Mat_ec(timestep/10+1,2) = ec2 - ec2_temp;
                ec1_temp = ec1;
                ec2_temp = ec2;
                %Leaving cars
                Mat_lc(timestep/10+1,1) = lc1 - lc1_temp;
                Mat_lc(timestep/10+1,2) = lc2 - lc2_temp;
                lc1_temp = lc1;
                lc2_temp = lc2;
            end
            end
        end
        
        %Percentage of the progress
        perpro = timestep/(5*N)*100;
        if(mod(perpro, 1) == 0)
            clc
            disp(['Progress: ' num2str(perpro) '%'])
        end
        
        if(time == simtime)
            avtc = av_time_c/lcc;
            avtt = av_time_t/ltc;
            disp(['The average transit time for a car is: ' num2str(avtc)]);
            disp(['The average transit time for a truck is: ' num2str(avtt)])
        end
    end
    
    Mat_ec = Mat_ec/(simtime/(5*N));
    Mat_lc = Mat_lc/(simtime/(5*N));
    
    if(withDensity)
        % plot density
        figure
        surf(x, time_vec, Mat_density1, 'EdgeColor', 'none');
        set(gca, 'Fontsize', 14)
        xlabel('position');
        ylabel('time');
        title('density lane 1 (vehicle/km)');
        colorbar;
        figure
        surf(x, time_vec, Mat_density2, 'EdgeColor', 'none');
        set(gca, 'Fontsize', 14)
        xlabel('position');
        ylabel('time');
        title('density lane 2 (vehicle/km)');
        colorbar;
    end
    
    if(withVelocity)
        % plot velocity
        figure
        hold on
        view(0,90);
        surf(x, time_vec, Mat_velocity1, 'EdgeColor', 'none');
        set(gca, 'Fontsize', 14)
        xlabel('position');
        ylabel('time');
        title('velocity lane 1 (m/s)');
        colorbar;
        figure
        hold on
        view(0,90);
        surf(x, time_vec, Mat_velocity2, 'EdgeColor', 'none');
        set(gca, 'Fontsize', 14)
        xlabel('position');
        ylabel('time');
        title('velocity lane 2 (m/s)');
        colorbar;
    end
    
    if(withVehicleCounter)
        %Incoming traffic flow
        ictf = sum(Mat_ec,1)/(5*N)*3600
        %Outgoing traffic flow
        ogtf = sum(Mat_lc,1)/(5*N)*3600
        %plot number of entering cars
        figure
        bar(time_vec2,Mat_ec, 'hist')
        xlabel('time', 'Fontsize', 14);
        ylabel('Number of entering cars per 10s', 'Fontsize', 14);
        title('incoming trafficflow', 'Fontsize', 16)
        legend('lane 1', 'lane 2')
        set(gca, 'XLim', [0 simtime], 'Fontsize', 12)
        %plot number of leaving cars
        figure
        bar(time_vec2,Mat_lc, 'hist')
        xlabel('time', 'Fontsize', 14);
        ylabel('Number of leaving cars per 10s', 'Fontsize', 14);
        title('outgoing trafficflow', 'Fontsize', 16)
        legend('lane 1', 'lane 2')
        set(gca, 'XLim', [0 simtime], 'Fontsize', 12)
    end
end