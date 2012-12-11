function [avtc, avtt, ictf, ogtf]=evaluate_simualtion(dir, simtime, l_highway)
%evaluate_simualtion(dir, simtime, l_highway)
%avtc: average transit time for a car
%avtt: average transit time for a truck
%ictf: incoming traffic flow [vehicle/h]
%ogtf: outgoing traffic flow [vehicle/h]
%dir: Direction

%Define Parameters
N = 100;                              %space resolution
x = linspace(0,l_highway,5*N+1);      %evaluating points
time_vec = linspace(0,simtime,5*N+1); %time vector
Mat_ec = zeros(N/2+1,2);              %Entering cars
Mat_lc = zeros(N/2+1,2);              %Leaving cars
ec1_temp = 0;
ec2_temp = 0;
lc1_temp = 0;
lc2_temp = 0;

    % Calculate density
    for timestep = 0:1:5*N
        % load statefile
        time = round(time_vec(timestep+1));
        load([dir '/statefile_' num2str(time)])
        if (mod(timestep,10) == 0)
            if (time)
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
        
        if(time == simtime)
            avtc = av_time_c/lcc;
            avtt = av_time_t/ltc;
        end
        
    end
    
    Mat_ec = Mat_ec/(simtime/(5*N));
    Mat_lc = Mat_lc/(simtime/(5*N));
    %Incoming traffic flow
    ictf = sum(Mat_ec,1)/(5*N)*3600;
    %Outgoing traffic flow
    ogtf = sum(Mat_lc,1)/(5*N)*3600;
            
end