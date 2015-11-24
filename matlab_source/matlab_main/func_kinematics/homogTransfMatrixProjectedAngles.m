
function [rotations] = homogTransfMatrixProjectedAngles(T, plotFigHandle)
% [rotations] = homogTransfMatrixProjectedAngles(T, plotFigHandle)
% [rotations] = homogTransfMatrixProjectedAngles(T, [])
%
% This function is useful to make a visual inspection on the homogeneous
% tranf. matrix.
% It plot the angles of all unity vectors of the reference frame of T in
% relation to the world frame.
%
% INPUT
%   T: 4x4 homog. transf. matrix
%   plotFigHandle: is a figure handle. If empty no figure will be plot.
%
% OUTPUT
%     angles.vectorX.aroundWorldXYZ = [ rot1 rot2 rot3 ]
%     angles.vectorY.aroundWorldXYZ = [ rot1 rot2 rot3 ]
%     angles.vectorZ.aroundWorldXYZ = [ rot1 rot2 rot3 ]
% 
% For example 
%     The 3 values in angles.vectorY.aroundWorldXYZ correspond to the
%     rotations in relation to the world axes X, Y, Z of the y vector of
%     the reference frame of T
%

    if ~isempty(plotFigHandle)
        
        param.hfig = plotFigHandle;        param.axis_length = 0.15;
        param.nMaxPlots = 10;
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'b', 'Marker', 'o');
        param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
        param.shadowHeight     = -1.22;
        ul = param.axis_length;

        refFrame(1).axis  = [ul 0 0 1]';
        refFrame(1).style = sty('r', [], 3);
        refFrame(2).axis  = [0 ul 0 1]';
        refFrame(2).style = sty('g', [], 3);
        refFrame(3).axis  = [0 0 ul 1]';
        refFrame(3).style = sty('b', [], 3);   
        Torigin = eye(4);
        tmp  = Torigin*[0 0 0 1]'; % get the position of the origin of the ref frame
                                    % which is the same as the position of
                                    % the point itself.

        title('rotations of each vector projection around world coordinates');
        
        % plot the world reference frame
        proj = [1 1 0; 1 0 1; 0 1 1];   orig = tmp(1:3)';
        for j=1:3
            out(j).val(1,:) = orig; 
            tmp = Torigin*refFrame(j).axis;
            out(j).val(2,:) = tmp(1:3)';
            plot3(     out(j).val(:,1), ...
                       out(j).val(:,2), ...
                       out(j).val(:,3), ...
                       refFrame(j).style );
        end        
        plotFlag=1;
    else
        ul = 0.02;
        refFrame(1).axis  = [ul 0 0 1]';
        refFrame(1).style = sty('r', [], 3);
        refFrame(2).axis  = [0 ul 0 1]';
        refFrame(2).style = sty('g', [], 3);
        refFrame(3).axis  = [0 0 ul 1]';
        refFrame(3).style = sty('b', [], 3);          
        plotFlag=0;
    end   
    
    rotations = plott(T, refFrame, plotFlag);    


end


function [rotations] = plott(T, refFrame, plotFig)

    tmp  = T*[0 0 0 1]'; % get the position of the origin of the ref frame
                                % which is the same as the position of
                                % the point itself.
    % plot the reference frame of T
    proj = [1 1 0; 1 0 1; 0 1 1];
    orig = tmp(1:3)';
    if plotFig
        for j=1:3
            out(j).val(1,:) = orig; 
            tmp = T*refFrame(j).axis;
            out(j).val(2,:) = tmp(1:3)';
            plot3(     out(j).val(:,1), ...
                       out(j).val(:,2), ...
                       out(j).val(:,3), ...
                       refFrame(j).style );
        end
    end
    
    % plot and fill the output structure
    % It plots the projection of the reference frame on all Cartesian
    % planes
    nameField = {'vectorX', 'vectorY', 'vectorZ'};    
    for j=1:3
        rotations.(nameField{j}).aroundWorldXYZ = [];            
        out(j).val(1,:) = orig; 
        tmp = T*refFrame(j).axis;
        out(j).val(2,:) = tmp(1:3)';
        
        for jj=1:3            
            if plotFig
                plot3(     proj(jj,1)*out(j).val(:,1), ...
                           proj(jj,2)*out(j).val(:,2), ...
                           proj(jj,3)*out(j).val(:,3), ...
                           sty([0.6 0.6 0.6]) );
            end            
            v = [proj(jj,1)*out(j).val(:,1) proj(jj,2)*out(j).val(:,2) proj(jj,3)*out(j).val(:,3)];
            vOrig = v(2,:)-v(1,:);
            if jj == 1
                ir = [1 2];
            end  
            if jj == 2
                ir = [1 3];
            end      
            if jj == 3
                ir = [2 3];
            end                  
            vOrig = vOrig(ir);  

            [d] = angle_between_vec( vOrig , [1 0] ) ;
            if vOrig(2) < 0
                d= -d;
            end
            rotations.(nameField{j}).aroundWorldXYZ(jj) = d;
            
            if plotFig % write the angle value close to the projection.
                text(v(2,1), v(2,2), v(2,3), num2str(r2d(d)));
            end
        end  
        rotations.(nameField{j}).aroundWorldXYZ = rotations.(nameField{j}).aroundWorldXYZ(3:-1:1);
        r2d( rotations.(nameField{j}).aroundWorldXYZ );   
    end

    
    
end




