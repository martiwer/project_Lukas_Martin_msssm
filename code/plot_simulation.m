function plot_simulation(simtime, l_highway_start, l_highway_end)
% plot_simulation(simtime, l_highway_start, l_highway_end)
% plots the traffic flow for the duration of simtime
% and the section of highway from l_highway_start to l_highway_end

%Indexes of the state matrix
ix = 1;         %Position
iv = 2;         %Velocity
iacc = 3;       %Acceleration
iv0 = 4;        %Desired velocity
iT = 5;         %Safe time headway
icp = 6;        %Changed position
itype = 7;      %Type 1:car; 2:truck
iol = 8;        %Original lane 

l_highway = l_highway_end - l_highway_start;

    for time = 0:1:simtime
        % load statefile
        load(['data/statefile_' num2str(time)])
            %plot street
                clf;
                y_dist=0.5;
                plot(l_highway_start:l_highway_end, 0*(0:l_highway), 'Color', [.75 .75 .75], 'LineWidth', 600)
                hold on;
                plot([l_highway_start,l_highway_end] ,[0,0],'--w','LineWidth', 1)
                hold on;
                plot([l_highway_start,l_highway_end] ,[-y_dist,-y_dist],'k','LineWidth', 2)
                hold on;
                plot([l_highway_start,l_highway_end] ,[y_dist,y_dist],'k','LineWidth', 2)
                xlim([l_highway_start l_highway_end])
                ylim([-1.5*y_dist 1.5*y_dist])
                xlabel('Position [m]')
             % plot cars
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
             % plot time
                text(l_highway_start + 4/5*l_highway, 1.15*y_dist, ['time= ' num2str(time)])
             pause(0.05)
    end
end