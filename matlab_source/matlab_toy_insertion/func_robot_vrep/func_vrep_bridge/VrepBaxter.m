classdef VrepBaxter < VrepAgent
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
       
    properties
        nHandDoF

        % isoemp stuff
        nTraj = 10;
        costIK
        costIK_clean

        nTrajConnect = 5;
        
        floorHeight = -1.220; % meters
        tableHeight = -0.475; % meters
        
        elbowConfig
        
        qmin
        qmax
    end

methods

    function obj = VrepBaxter(vrepObject)
        
        obj@VrepAgent(vrepObject, 'Baxter');
        obj.getJointHandles({'Baxter_rightArm_joint1', 'Baxter_rightArm_joint2', 'Baxter_rightArm_joint3', 'Baxter_rightArm_joint4', ...
                            'Baxter_rightArm_joint5', 'Baxter_rightArm_joint6', 'Baxter_rightArm_joint7'});

        try
            obj.getJointHandlesFKarm({'Baxter_rightArm_joint1FK', 'Baxter_rightArm_joint2FK', 'Baxter_rightArm_joint3FK', 'Baxter_rightArm_joint4FK', ...
                               'Baxter_rightArm_joint5FK', 'Baxter_rightArm_joint6FK', 'Baxter_rightArm_joint7FK'});
        catch
            fprintf('\n\n *** this Baxter model does not have a FK implementation ***\n\n');
        end
        obj.nDoF = 7;
        
        % store elbow configuration
        load('elbowInitConfig.mat');
        obj.elbowConfig = elbow;
        
        [~, obj.qmax, obj.qmin] = aux_useful_baxter_info();
        obj.qmin = d2r(obj.qmin); obj.qmax = d2r(obj.qmax);
        
    end   

    function openFiles(obj)
        edit('VrepAgent.m');
        edit('VrepBaxter.m');
    end
    
    function setCartesian(obj, T, dummy_name)
        % robot.setCartesian(T, 'Dummy_target')
        obj.sendTargetCartesianCoordinates(T(1:3,4), tr2rpy(T), obj.getHandle(dummy_name), 1)
    end
    
    function [obj] = setRestPosture(obj, startElbowConfig )
        switch startElbowConfig
            case 'elbow_up'
                obj.qRestPosture = obj.elbowConfig.up.qRest;
                obj.TrestPosture = obj.elbowConfig.up.restT;
            case 'elbow_down'
                obj.qRestPosture = obj.elbowConfig.down.qRest;
                obj.TrestPosture = obj.elbowConfig.down.restT;
        end
    end

    function [limitHitMin, limitHitMax] = checkJointLimit(obj, q, checkPlot)
       limitHitMin  = ~(obj.qmin < min(q));
       limitHitMax  = ~(obj.qmax > max(q));
       
       if checkPlot
           if sum(limitHitMin) > 0 || sum(limitHitMax) > 0
               c_ = distinguishable_colors(7);
               figurew('jointLimits');
               for j=1:7
                    subplot(4,2,j); grid on; hold on;
                    plot(q(:,j), sty(c_(j,:), [], 2) );
                    text(numel(q(:,j))+1, q(end,j), num2str(j) );
                    plot( [1:numel(q(:,1))], ones(1, numel(q(:,1))).*obj.qmin(j),  sty(c_(j,:), [], 2, '--') );
                    plot( [1:numel(q(:,1))], ones(1, numel(q(:,1))).*obj.qmax(j),  sty(c_(j,:), [], 2, '--') );
               end
           end
       end
       
    end
    
    function restart(obj)

        obj.costIK=[];
        obj.costIK_clean=[];

    end
    
    function [ikerrorTraj, h, T2, q] = IKcost(obj, j, flagClean, h, T)

        % quickly resample the trajectory to make IK faster
        dt =  numel(T(1,1,:))/obj.nTraj  ;

        if dt >= 1
            % the number of points will be certainly
            % decreased to make IK faster. This may not be the case if you want
            % to replay the trajector slowly
            % =======================================================            
            dt = round(dt);
            
            id = 1:dt:numel(T(1,1,:));
            if id(end)~=numel(T(1,1,:))
                id(end+1) =numel(T(1,1,:));
            end
            T2 = T(:,:,id);

            if ~isempty(obj.nTrajConnect)     % add connecting trajectory
                T_connect = obj.goTo(obj.TrestPosture, T2(:,:,1), obj.nTrajConnect);
                T2 = cat(3, T_connect, T2);
            end
        end
        
        if dt < 1 
            % this is only used to increase the trajectory size beyond 
            % the initial demonstration. Useful to replay slowly or
            % to resample. interpolate quaternions
            dt = 1;
            id = 1:dt:numel(T(1,1,:));
            if id(end)~=numel(T(1,1,:))
                id(end+1) =numel(T(1,1,:));
            end
            T2 = T(:,:,id);
            if ~isempty(obj.nTrajConnect)     % add connecting trajectory
                T_connect = obj.goTo(obj.TrestPosture, T2(:,:,1), obj.nTrajConnect);
                T2 = cat(3, T_connect, T2);
            end
            
            % extra step: resample to higher samplings
            for t=1:numel(T2(1,1,:))
                q_ = Quaternion(T2(:,:,t));
                pose(t,:) = [T2(1:3,4,t)' q_.s q_.v];
            end
            pose_ = interp1(linspace(0,1,numel(pose(:,1))),  pose, linspace(0,1, obj.nTraj  ));
            for t=1:obj.nTraj
                T2(:,:,t) = quaternion2homogTransfMatrix( pose_(t,4:end) ) + [zeros(4,3)  [pose_(t,1:3)'; 0] ];
            end
            for t = 1:10 % repeat a few times to allow IK to damp
                T2(:,:,end+1) = T2(:,:,end);
            end
            
        end

        ik_hist = obj.IK(T2, 0);        
        q = ik_hist.q;
        obj.costIK(j) = sum(ik_hist.ne.^2);
        

        if flagClean == 0
            obj.costIK_clean      = [obj.costIK_clean     obj.costIK(j)]; 
            figure(h.fig);
            xyzClean = squeeze(ik_hist.T(1:3,4,:));
            h.plotCartIK = [h.plotCartIK plot3(xyzClean(1,:), xyzClean(2,:), xyzClean(3,:), sty('b', [], 2))];

     
            param.hfig = h.fig;
            param.axis_length = 0.03;
            param.nMaxPlots = 5;
            param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'r', 'Marker', 'none');
            param.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', [1 0 0]);
            param.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', [0 1 0]);
            param.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', [.0 .0 1]);
            [~, htmp] = homogTransfPlot(T2, param);
            h.plotCartIK = [h.plotCartIK htmp];
            
        end
        ikerrorTraj = ik_hist.ne;            
    end

    function [xyz, rpy, To1, qrobot, qhandCell] = readGenericCoordinates(obj, indexDummy)
    % function [xyz, rpy, To1, q] = readGenericCoordinates(obj, indexDummy)
    % Example: [xyz, rpy, To1, q] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'))
    %
    % indexDummy is the index, *not* the handle!;
    % 
    % This function is specific to the scen of Darias + human.

        % Read tip position via the dummy attached to it
        dataTypeDummy = 9;  % 9: object position and orientation (Euler angle)
        vrep = obj.vrep;
        
        % this one has to be simx_opmode_oneshot_wait so that we force it to get fresh data
        [~,~,~,dummyOut]=...
            vrep.simxGetObjectGroupData(obj.clientID, vrep.sim_object_dummy_type,...
                               dataTypeDummy, vrep.simx_opmode_oneshot_wait);                   

        % parse dummy coordinates
        block_size = 6; % xyz rpy
        istart   =  (indexDummy-1)*block_size+1;
        iend     =  istart+2;
        xyz = double(dummyOut(istart:iend));

        istart = iend+1;
        iend = istart+2;
        rpy = double(dummyOut(istart:iend));

        % create a homog To1
        T = [ [eye(3); [0 0 0]] [xyz 1]' ];     
        param.center_of_rotation = xyz;  param.plot_flag = 0;   param.rotation_seq = 'rpy';
        [To1] = homogTransfRotate( rpy, T, param );

        % Read the joints of the arm
        if nargout > 3
            
            % since we already forced fresh data before, you can just get the next one from the buffer
            dataTypeJoint = 15; % 15: retrieves joint state data (in floatData (2 values): position, force/torque)
            getJointFlag = -999;
            while getJointFlag ~=0
                [getJointFlag, ~, ~, jointOut]=  vrep.simxGetObjectGroupData(...
                    obj.clientID, vrep.sim_object_joint_type, dataTypeJoint , vrep.simx_opmode_buffer);
            end
            
            % parse joint coordinates
            iStart = obj.joint.index(1); % the index of the first joint
            iEnd   = obj.joint.index(end); % the index of the last joint

            iStart2 = (iStart)*2-1;
            iEnd2 = (iEnd)*2-1;
            
            q = double(jointOut(iStart2:2:iEnd2));
            qrobot = q(1:obj.nDoF);

            if nargout > 4
                qhand = q(obj.nDoF+1:obj.nDoF+obj.nHandDoF);
                
                % return the hand joint values as cells
                qhandCell{1} = [qhand(17)  qhand(1) qhand(1+4)  qhand(1+2*4) qhand(1+3*4)];
                qhandCell{2} = [qhand(18)  qhand(2) qhand(2+4)  qhand(2+2*4) qhand(2+3*4)];
                qhandCell{3} = 2*[qhand(19)  qhand(3) qhand(3+4)  qhand(3+2*4) qhand(3+3*4)];
                
                % check that the distal+tip are half-half
                if qhand(19)~=qhand(20)
                   fprintf('Thumb\n');
                   fprintf('Distal and tip angles are not the same\n');
                   keyboard
                end
                if qhand(3)~=qhand(4)
                   fprintf('Index\n');
                   fprintf('Distal and tip angles are not the same\n');
                   keyboard       
                end   
                if qhand(7)~=qhand(8)
                   fprintf('Ring\n');
                   fprintf('Distal and tip angles are not the same\n');
                   keyboard       
                end 
                if qhand(11)~=qhand(12)
                   fprintf('Middle\n');
                   fprintf('Distal and tip angles are not the same\n');
                   keyboard       
                end 
                if qhand(15)~=qhand(16)
                   fprintf('Small\n');
                   fprintf('Distal and tip angles are not the same\n');
                   keyboard       
                end                     
            end
        end
    end
      
    function [qDarias] = getJointAngles(obj)        
        [~, ~, ~, qDarias] = obj.readGenericCoordinates(obj.getIndex('Dummy_tip'));
    end
    
    function [] = createTableDariasFingerName(obj)
    % create an easier lookup table for the hands of Darias.
        
        spreadNames = {'right_hand_thumb_spread_joint', ...
                       'right_hand_index_finger_spread_joint', ...
                       'right_hand_middle_finger_spread_joint',...
                       'right_hand_ring_finger_spread_joint',...
                       'right_hand_small_finger_spread_joint' };
        tbl =[];
        for j=1:numel(spreadNames)
            handle = obj.joint.handle(strcmp(obj.joint.name, spreadNames{j}));
            tbl = [tbl; {[j], [handle], spreadNames{j}}];
        end
        obj.spreadTableFinger = tbl;
        
        proximalNames = {'right_hand_thumb_proximal_joint', ...
                         'right_hand_index_finger_proximal_joint', ...
                         'right_hand_middle_finger_proximal_joint',...
                         'right_hand_ring_finger_proximal_joint',...
                         'right_hand_small_finger_proximal_joint' };
        tbl =[];
        for j=1:numel(spreadNames)
            handle = obj.joint.handle(strcmp(obj.joint.name, proximalNames{j}));
            tbl = [tbl; {[j], [handle], spreadNames{j}}];
        end
        obj.proximalTableFinger = tbl;
        
        
        distalNames = {'right_hand_thumb_distal_joint', ...
                         'right_hand_index_finger_distal_joint', ...
                         'right_hand_middle_finger_distal_joint',...
                         'right_hand_ring_finger_distal_joint',...
                         'right_hand_small_finger_distal_joint' };
        tbl =[];
        for j=1:numel(spreadNames)
            handle = obj.joint.handle(strcmp(obj.joint.name, distalNames{j}));
            tbl = [tbl; {[j], [handle], spreadNames{j}}];
        end
        obj.distalTableFinger = tbl;
        
        
        tipNames = {'right_hand_thumb_tip_joint', ...
                         'right_hand_index_finger_tip_joint', ...
                         'right_hand_middle_finger_tip_joint',...
                         'right_hand_ring_finger_tip_joint',...
                         'right_hand_small_finger_tip_joint' };
        tbl =[];
        for j=1:numel(spreadNames)
            handle = obj.joint.handle(strcmp(obj.joint.name, tipNames{j}));
            tbl = [tbl; {[j], [handle], spreadNames{j}}];
        end
        obj.tipTableFinger = tbl;

    end
    
    function [] = help(obj)
        disp('SET A CARTESIAN POSITION FOR A DUMMY')
        disp(' sendTargetCartesianCoordinates(obj, xyz, rpy, handleDummy, one_shot_wait_flag)');
        disp(' robot.sendTargetCartesianCoordinates(T(1:3,4,k), tr2rpy(T(:,:,k)), robot.getHandle("Dummy_target"), 1)');
        
        disp('READ A CARTESIAN POSITION')
        disp('   [T] = robot.readEntityCoordinate("Dummy_tipFK");')
    end
    
end
    
    

    
end
