classdef Darias_ik < handle
       
    properties
        para
        nTraj = 10;
        restPostureT
        paramIK
        costIK
        costIK_clean
        rob
        opt
    end

methods
    
    function obj = Darias_ik(vrepObject)
        obj.rob = VrepDarias(vrepObject);
    end
    
    function [ikerrorTraj, h] = IKcost(obj, j, flagClean, h, xyz, refOrientation)
                             %  j, (j-nRollOut), h, DMP.XYZrollOut(:,:,:,j), DMP.refOrientation

        % quickly resample the trajectory to make IK faster
        dt = round(numel(xyz(1,:))/obj.nTraj);
        id = 1:dt:numel(xyz(1,:));
        if id(end)~=numel(xyz(1,:))
            id(end+1) = numel(xyz(1,:));
        end
        k=1;
        for i=id
            T2(:,:,k) =  [   [refOrientation(1:3,1:3, i);[0 0 0]]    [xyz(:,i);1]];
            k=k+1;
        end
        
        % add connecting trajectory
        T_connect = obj.rob.goTo(obj.rob.TrestPosture, T2(:,:,1), 5);
        T2 = cat(3, T_connect, T2);
        obj.rob.IK(T2, []);
        
        



        %homogTransfPlot(T2, struct('hfig', h.fig))


        % =====================================
        % ik on the currentT    
        qi = obj.para.restAngles';    
        ik_hist =  obj.my_ikine(T2, obj.para.restAngles');

        if 0    
            figure(h.fig);
            h.del = []; 
            nTraj = numel(ik_hist.T(1,1,:));
            for k=1:nTraj
                [T_, p_ ] = obj.FK_and_homog_transf( ik_hist.q(k,:) );
                h.del = [h.del  plot(p_(:,  1), p_(:,  2), styw('k', 'o', 4,' -',10 )  )];
                title(['Step ' num2str(k) ' of '  num2str(nTraj)]);       
                drawnow;
                pause(0.4) %keyboard
                if  1% k ~= numel(traj.q_ik(:,1))
                    delete(h.del); h.del=[];
                end
            end                
        end


        % =====================================
        %ik_hist.neResampled = interp1(linspace(0,1, numel(ik_hist.ne)),ik_hist.ne,   linspace(0,1,param.ikine.nResampleError));
        obj.costIK(j) = sum(ik_hist.ne.^2);

        if flagClean == 0
            obj.costIK_clean      = [obj.costIK_clean     obj.costIK(j)]; 
            figure(h.fig);
            xyzClean = squeeze(ik_hist.T(1:3,4,:));
            h.plotCartIK = [h.plotCartIK plot3(xyzClean(1,:), xyzClean(2,:), xyzClean(3,:), sty('b', [], 2))];

        end
        ikerrorTraj = ik_hist.ne;            
    end

end

end