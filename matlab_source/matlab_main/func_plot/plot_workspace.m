
function hfig2 = plot_workspace()

    hfig2 = figurewe('main');    
    
    [xyz, cz]= get_table_coordinates();
    
    
    plot3(xyz(:,1), xyz(:,2), xyz(:,3), sty_nl('k',[],2,'-',[])) ;
    plot3(0, 0, cz, styw_nl('k', 'o', 2, '-', 10)) ;

    view([-1 -1 1.5]);
    xlabel 'x(m)'; ylabel 'y(m)'; zlabel 'z(m)';
    
    
    
end
