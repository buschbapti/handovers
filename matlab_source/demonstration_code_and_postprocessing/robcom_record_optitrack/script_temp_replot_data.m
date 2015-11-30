

function [] = fjfi(dataMocap, scale)

figurew; xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';


param.hfig = gcf;
param.axis_length = scale*0.15;
param.nMaxPlots =  50;
param.pathPlotStyle =  styw([0.6 0.6 0.6], 'o', 0.25, [], 5);
homogTransfPlot(dataMocap.robotT, param);
homogTransfPlot(dataMocap.humanT, param);
homogTransfPlot(dataMocap.capT, param);
axis equal
set_fig_position([0.486  0 0.485 0.934]); 
view([0 0 1 ])
text(dataMocap.robotT(1,4,1), dataMocap.robotT(2,4,1), dataMocap.robotT(3,4,1), 'robot' );
text(dataMocap.humanT(1,4,1), dataMocap.humanT(2,4,1), dataMocap.humanT(3,4,1), 'human' );
text(dataMocap.capT(1,4,1), dataMocap.capT(2,4,1), dataMocap.capT(3,4,1), 'cap' );
