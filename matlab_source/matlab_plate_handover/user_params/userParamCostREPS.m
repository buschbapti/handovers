function  repsCost = userParamCostREPS( )

    repsCost.accel = 1;
    repsCost.vel = 0;
    repsCost.viaPoint  = 10000;
    repsCost.viaPointVelocity  = 0;
    
    repsCost.startGoal = 0;
    repsCost.paramSize = 0;
    
    repsCost.deviationFromMean = 0.1;
    
    repsCost.collision = 0;    
    
    repsCost.odometryCartesian = 0.1;

    
    
end

