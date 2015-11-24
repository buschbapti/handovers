function posesFromROS = calibrate_ref_TUDa_Inria_ref_frame(posesFromROS)
% Only calibrates xyz, hopefully the orientation does not need to be
% calibrated as the axes seem to point to the same directions.


    %% Adjust XYZ

    % I think the XY of both Darias and Baxter are matching. But the height
    % of the reference frame is not the same.
    xyzShift = [0  0  -0.3543];
    posesFromROS(:,1:3) = bsxfun(@plus, posesFromROS(:,1:3), xyzShift);
    
    
    %% Ajust quaternions
    %  hopefully not necessary. If necessary, it means the reference frames
    %  are rotated in relation to each other???   
    

    

end