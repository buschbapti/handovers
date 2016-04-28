classdef VrepDarias < VrepAgent
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
       
    properties
        spreadTableFinger   ='used for Darias';
        tipTableFinger      ='used for Darias';
        proximalTableFinger ='used for Darias';
        distalTableFinger   ='used for Darias';
        nHandDoF
        
        % isoemp stuff
        nTraj = 10;
        costIK
        costIK_clean

        nTrajConnect = 5;
        
        floorHeight = -1.220; % meters
        tableHeight = -0.475; % meters        
    end

methods

    function obj = VrepDarias(vrepObject)
        
         obj@VrepAgent(vrepObject, 'Darias');
         
         obj.getJointHandles({'LBR4p_joint1', 'LBR4p_joint2', 'LBR4p_joint3', 'LBR4p_joint4', ...
                             'LBR4p_joint5', 'LBR4p_joint6', 'LBR4p_joint7', ...
                            'right_hand_thumb_spread_joint',...
                            'right_hand_thumb_proximal_joint',...
                            'right_hand_thumb_distal_joint',...
                            'right_hand_thumb_tip_joint',...                                              
                            'right_hand_index_finger_spread_joint',...
                            'right_hand_index_finger_proximal_joint',...
                            'right_hand_index_finger_distal_joint',...
                            'right_hand_index_finger_tip_joint',...
                            'right_hand_middle_finger_spread_joint',...
                            'right_hand_middle_finger_proximal_joint',...
                            'right_hand_middle_finger_distal_joint',...
                            'right_hand_middle_finger_tip_joint',...
                            'right_hand_ring_finger_spread_joint',...
                            'right_hand_ring_finger_proximal_joint',...
                            'right_hand_ring_finger_distal_joint',...
                            'right_hand_ring_finger_tip_joint',...
                            'right_hand_small_finger_spread_joint',...
                            'right_hand_small_finger_proximal_joint',...
                            'right_hand_small_finger_distal_joint',...
                            'right_hand_small_finger_tip_joint',...                            
                                });
        obj.nDoF = 7;
        obj.nHandDoF = 20;

        obj.createTableDariasFingerName();         
        
    end   
    
    function openFiles(obj)
        edit('VrepAgent.m');
        edit('VrepDarias.m');
    end

    function restart(obj)

        obj.costIK=[];
        obj.costIK_clean=[];

    end
    
    function [ikerrorTraj, h, q] = IKcost(obj, j, flagClean, h, T)

        % quickly resample the trajectory to make IK faster
        dt = round(numel(T(1,1,:))/obj.nTraj);
        id = 1:dt:numel(T(1,1,:));
        if id(end)~=numel(T(1,1,:))
            id(end+1) =numel(T(1,1,:));
        end
        T2 = T(:,:,id);
          
        if ~isempty(obj.nTrajConnect)     % add connecting trajectory
            T_connect = obj.goTo(obj.TrestPosture, T2(:,:,1), obj.nTrajConnect);
            T2 = cat(3, T_connect, T2);
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

    
    function [] = setFingersDefaultPos(obj)        
        spread   = d2r([10  0  0  0  0]);
        proximal = 1*d2r([5 5 5 5 5 ]); % range 5 to 80
        distal   = 1*d2r([10 10 10 10 10]); %range 10 to 130                
        obj.setDariasHandJointAngles({spread, proximal, distal});                
    end    

    function [] = setDariasHandJointAngles(obj, qCell)
    % Define positions of the finger of Darias
    % qCell is three cells of angles given in radians
    %            [ thum, index, middle, ring, small  ] 
    % qCell{1} = [   q1,    q2,     q3,   q4,    q5  ] are the spread values
    % qCell{2} = [   q1,    q2,     q3,   q4,    q5  ] are the proximal angle values
    % qCell{3} = [   q1,    q2,     q3,   q4,    q5  ] are the total angles
    %             achieved by the q_distal + q_tip angles of the hand. This
    %             is because both q_distal and q_tip are controlled by the
    %             same motor. If you specify a value qCell{3}(j) that will
    %             be sent as  q_distal =  qCell{3}(j)/2
    %                         q_tip    =  qCell{3}(j)/2
    % 
    % If you do not know the current joint angle of certain joints and do
    % not care about them, but want to send new joint angles for other
    % joints use NaN. Entries with NaN will be ignored and the current
    % angles will be maintained.
    
               
        jointHandle = [];
        jointAngle  = [];
        
        % check spread angles
        ci=1;
        for j=1:numel(qCell{ci})
            if ~isnan(qCell{ci}(j))
                jointHandle = [jointHandle;
                      obj.spreadTableFinger{j,2}]; 
                jointAngle = [jointAngle; qCell{ci}(j) ];
            end
        end        
        
        % check proximal
        ci=2;
        for j=1:numel(qCell{ci})
            if ~isnan(qCell{ci}(j))
                jointHandle = [jointHandle ;
                      obj.proximalTableFinger{j,2}]; 
                jointAngle = [jointAngle; qCell{ci}(j) ];
            end
        end           
        
        % check distalTableFinger
        ci=3;
        for j=1:numel(qCell{ci})
            if ~isnan(qCell{ci}(j))
                jointHandle = [jointHandle ;
                      obj.distalTableFinger{j,2}]; 
                jointAngle = [jointAngle; qCell{ci}(j)*0.5 ];
            end
        end
        
        % check tip
        ci=3;
        for j=1:numel(qCell{ci})
            if ~isnan(qCell{ci}(j))
                jointHandle = [jointHandle ;
                      obj.tipTableFinger{j,2}]; 
                jointAngle = [jointAngle; qCell{ci}(j)*0.5 ];
            end
        end
                
        nJointsToUpdate = numel(jointHandle);
        for j=1:nJointsToUpdate
            if j~=nJointsToUpdate
                obj.vrep.simxSetJointPosition(obj.clientID, jointHandle(j), jointAngle(j),  obj.vrep.simx_opmode_oneshot);
            else
                obj.vrep.simxSetJointPosition(obj.clientID, jointHandle(j), jointAngle(j),  obj.vrep.simx_opmode_oneshot_wait);
            end
        end
                
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
    
end
    
    

    
end
