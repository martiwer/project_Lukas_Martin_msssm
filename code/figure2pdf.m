% FIGURE2PDF(NAME)
% Exports current figure as PDF file
%
% NAME: Filename (without extension)
%
% RELab - ETHZ - 2010

function figure2pdf(name)

set(gcf, 'Units', 'centimeters');
pos=get(gcf,'Position');
size=[0 0 pos(3) pos(4)];


% Set the necessary dimensions
set(gcf,'PaperUnits', 'centimeters',...
        'PaperSize', size(3:4),...
        'PaperPositionMode', 'manual',...
        'PaperPosition', size,...
        'Renderer', 'painters');


% export in PDF format
print(gcf, '-dpdf', [name '.pdf']);