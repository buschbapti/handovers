
function plot_workspace(hfig)

    figure(hfig)
    
    [xyz, cz]= get_table_coordinates();
    
    
    plot3(xyz(:,1), xyz(:,2), xyz(:,3), sty_nl('k',[],2,'-',[])) ;

    view([-1 -1 1.5]);
    xlabel 'x(m)'; ylabel 'y(m)'; zlabel 'z(m)';
    
    
    z = linspace(-0.47,0,10);
    x = 0*linspace(-0.47,0,10);
    y = 0*linspace(-0.47,0,10);
    plot3(x, y, z, sty_nl('k',[],2,'-',[])) ;
    
    
    
end
