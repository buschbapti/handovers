function [C, Cparts, h] = cost(xyz, h, vp, letterAr, obst, flagClean)

    dbg = 1;
    if dbg
        figure(h.fig)
        if flagClean ~= 0
           % h.plot = [h.plot plot3(xyz(1,:), xyz(2,:), xyz(3,:), sty(lightRGB(1),[],1))];
        else
            h.plot = [h.plot plot3(xyz(1,:), xyz(2,:), xyz(3,:), sty('r',[],2))];
        end
    end
    
    
    % compare similarity here {R}
    Csimilarity = similarity(letterAr, xyz);
    Csimilarity = 1*Csimilarity.^2;
    
    
    % compute obstacle avoidance
    Cobstacle = 100*obstacle_avoid(xyz, obst);

    % via point
    VPerror(1) = sqrt(sum(    (vp.init' - xyz(1:3,1)').^2     )  );
    VPerror(2) = sqrt(sum(    (vp.finl'- xyz(1:3,end)').^2   )  );
    
    CviaPoint = 1000*mean( VPerror ).^2;

    C =  CviaPoint + Csimilarity + Cobstacle;
    
    Cparts.viaPoint   = CviaPoint;
    Cparts.similarity = Csimilarity;
    Cparts.obstacle   = Cobstacle;
    
end


function Csimilarity = similarity(trajA, trajB)

    distTraj =  sqrt( sum( (trajA-trajB).^2 ) ) ;
    
    %Csimilarity = mean(distTraj);
    Csimilarity = sum(distTraj.^2);



end

function Cobstacle = obstacle_avoid(x, obst )

   distanceToObstacle = bsxfun(@minus, x, obst.xyz)';
   distanceToObstacle = sqrt(sum(distanceToObstacle.^2,2));
   distanceToObstacle = max(  obst.rad-distanceToObstacle, 0   );
   
   Cobstacle = sum(distanceToObstacle.^2);
   
end
















