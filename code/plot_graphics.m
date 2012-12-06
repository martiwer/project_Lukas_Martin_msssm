function plot_graphics(simtime, l_highway_start, l_highway_end)
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
x = linspace(l_highway_start,l_highway_end,N+1);        %evaluating point
Mat_density1 = zeros(simtime+1,N+1);  %Spatiotemporal density Matrix lane 1
Mat_density2 = zeros(simtime+1,N+1);  %Spatiotemporal density Matrix lane 2

    % Calculate density
    for time = 0:1:simtime
        % load statefile
        load(['data/statefile_' num2str(time)])
        for place = 1:N+1
            counter = 0;
            for vehicle_1 = 1:size(state_1,1)
                if (state_1(vehicle_1,ix) < x(place) - 500)
                    break
                end
                if (state_1(vehicle_1,ix) < x(place) + 500)
                    counter = counter + 1;
                end                
            end
            Mat_density1(time+1,place) = counter;
            counter = 0;
            for vehicle_2 = 1:size(state_2,1)
                if (state_2(vehicle_2,ix) < x(place) - 500)
                    break
                end
                if (state_2(vehicle_2,ix) < x(place) + 500)
                    counter = counter + 1;
                end                
            end
            Mat_density2(time+1,place) = counter;
        end
    end
    
    % plot
    figure
    subplot(2,1,1)
    surf(Mat_density1);
    xlabel('position');
    ylabel('time');
    zlabel('density (vehicle/km)');
    subplot(2,1,2)
    surf(Mat_density2);
    xlabel('position');
    ylabel('time');
    zlabel('density (vehicle/km)');
end