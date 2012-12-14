%Simulation file
close all;
clear all;
clc;
%Denomination of saved files:
%Statefile_ d2_ sl_ n_ time

N = 20;                 %Number of simulations
simtime = 2000;         %Simulation time
l_highway = 10000;      %Length of simulated highway
avtc_Mat = zeros(20,3); %Matrix: Average transit time car
avtt_Mat = zeros(20,3); %Matrix: Average transit time truck
ictf_Mat = zeros(20,6); %Matrix: Incoming traffic flow
ogtf_Mat = zeros(20,6); %Matrix: Outgoing traffic flow


%Disturbance on (dist_factor = 0.6)
d = 2;                      %Disturbancetype
speed_limit = [0 100 80];


for k=1:size(speed_limit,2)
    sl = speed_limit(k);
    for n=1:20
        dir = ['data/dataset_d2_sl' num2str(sl) '_' num2str(n)];
        mkdir(dir)
        simulate_main(simtime,l_highway,d,sl,dir,0)
        load([dir '/statefile_' num2str(simtime)])
        if(crash == 0)
            [avtc, avtt, ictf, ogtf] = evaluate_simulation(dir, 2000, 10000);
            avtc_Mat(n,k) = avtc;
            avtt_Mat(n,k) = avtt;
            ictf_Mat(n,2*k-1:2*k) = ictf;
            ogtf_Mat(n,2*k-1:2*k) = ogtf;
        end
    end
end