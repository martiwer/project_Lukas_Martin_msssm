function plot_graphics(simtime, l_highway_start, l_highway_end, withDensity, withVelocity)
% plot_graphics(simtime, l_highway_start, l_highway_end)
% plots the traffic density (vehicles/km) and the average speed
% over a section of highway from l_highway_start to l_highway_end

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
N = 100;                              %space resolution
x = linspace(l_highway_start,l_highway_end,5*N+1); %evaluating points
time_vec = linspace(0,simtime,5*N+1);               %time vector
Mat_density1 = zeros(5*N+1,5*N+1);  %Spatiotemporal density Matrix lane 1
Mat_density2 = zeros(5*N+1,5*N+1);  %Spatiotemporal density Matrix lane 2
Mat_velocity1 = zeros(5*N+1,5*N+1); %Spatiotemporal velocity Matrix lane 1
Mat_velocity2 = zeros(5*N+1,5*N+1); %Spatiotemporal velocity Matrix lane 2

    % Calculate density
    for timestep = 0:1:5*N
        % load statefile
        time = round(time_vec(timestep+1));
        load(['data/statefile_' num2str(time)])
        if (withDensity)
            for place = 1:(5*N+1)
                counter = 0;
                for vehicle_1 = 1:size(state_1,1)
                    if (state_1(vehicle_1,ix) < x(place) - 500)
                        break
                    end
                    if (state_1(vehicle_1,ix) < x(place) + 500)
                        counter = counter + 1;
                    end                
                end
                Mat_density1(timestep+1,place) = counter;
                counter = 0;
                for vehicle_2 = 1:size(state_2,1)
                    if (state_2(vehicle_2,ix) < x(place) - 500)
                        break
                    end
                    if (state_2(vehicle_2,ix) < x(place) + 500)
                        counter = counter + 1;
                    end                
                end
                Mat_density2(timestep+1,place) = counter;
            end
        end
        if (withVelocity)
            for place = 1:(5*N+1)
                counter = 0;
                av_v = 0;
                for vehicle_1 = 1:size(state_1,1)
                    if (state_1(vehicle_1,ix) < x(place) - 100)
                            break
                    end
                    if (state_1(vehicle_1,ix) < x(place) + 100)
                            counter = counter + 1;
                            av_v = av_v + state_1(vehicle_1,iv);
                    end
                end
                if (counter)
                    Mat_velocity1(timestep+1,place) = av_v/counter;
                else
                    Mat_velocity1(timestep+1,place) = 0;
                end
                counter = 0;
                av_v = 0;
                for vehicle_2 = 1:size(state_2,1)
                    if (state_2(vehicle_2,ix) < x(place) - 100)
                            break
                    end
                    if (state_2(vehicle_2,ix) < x(place) + 100)
                            counter = counter + 1;
                            av_v = av_v + state_2(vehicle_2,iv);
                    end
                end
                if (counter)
                    Mat_velocity2(timestep+1,place) = av_v/counter;
                else
                    Mat_velocity2(timestep+1,place) = 0;
                end
            end
        end
        
        %Percentage of the progress
        perpro = timestep/(5*N)*100;
        if(mod(perpro, 1) == 0)
            clc
            disp(['Progress: ' num2str(perpro) '%'])
        end
    end
    
    if (withDensity)
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
    
    if (withVelocity)
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
end