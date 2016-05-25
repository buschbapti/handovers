function [] = plot_obstacle(obst, stl)

    t = 0:0.1:2*pi;
    
    x = obst.rad*sin(t);
    y = obst.rad*cos(t);
    

    plot3( obst.xyz(1)+x,   obst.xyz(2)+y, obst.xyz(3)+0.*y, stl);
    plot3( obst.xyz(1)+0*x, obst.xyz(2)+y, obst.xyz(3)+x, stl);
    plot3( obst.xyz(1)+x,   obst.xyz(2)+0*y, obst.xyz(3)+y, stl);

end