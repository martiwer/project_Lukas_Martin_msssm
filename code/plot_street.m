function plot_street
N=10000;
y1=0;
y2=1;
        plot(0:N, y1*(0:N), 'Color', [.75 .75 .75], 'LineWidth', 400)
        hold on;
        plot([0,N] ,[0,0],'--w','LineWidth', 1)
        hold on;
        plot([0,N] ,[-y2,-y2],'k','LineWidth', 2)
        hold on;
        plot([0,N] ,[y2,y2],'k','LineWidth', 2)
        xlim([0 N])
        ylim([-1.5*y2 1.5*y2])
end