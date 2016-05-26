classdef VrepAgent < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
       
    properties
        robotName =[];
        agentName  = [];
        vrep  = [];
        clientID = [];
        entt=[];
        joint=[];
        jointFK = []; % THE EXTRA ARM THAT IS ONLY USED FOR FK
        opt=[];
        qRestPosture;
        TrestPosture
        Treba
        initialCartesianTrajectory; % holds the initial guess.obj
        nDoF;
    end

methods

    function obj = VrepAgent(vrepObject, agentName )

        % Not sure if I can inherit the class Vrep directly here because it
        % will be used by multiple instances of VrepAgent. So I prefer to
        % simply copy the properties of the "parent object".
        obj.vrep      = vrepObject.vrep;
        obj.clientID  = vrepObject.clientID;
        obj.robotName = vrepObject.robotName;
        obj.agentName      = agentName;
        
        obj.entt.name   = [];
        obj.entt.handle = [];
        obj.entt.index  = [];
        
        obj.joint.name   = [];
        obj.joint.handle = [];
        obj.joint.index  = [];
        
        obj.jointFK.name   = [];
        obj.jointFK.handle = [];
        obj.jointFK.index  = [];        
        
        obj.opt = vrepObject.opt;
        
        
        switch obj.robotName 
            case 'LBR4'
                obj.getJointHandles({'LBR4p_joint1', 'LBR4p_joint2', 'LBR4p_joint3', 'LBR4p_joint4', ...
                                        'LBR4p_joint5', 'LBR4p_joint6', 'LBR4p_joint7'});               
                obj.nDoF = 7;
            case 'UR5'
                error('todo')                
            case 'Baxter'
                
                obj.getJointHandles({'Baxter_rightArm_joint1', 'Baxter_rightArm_joint2', 'Baxter_rightArm_joint3', 'Baxter_rightArm_joint4', ...
                                        'Baxter_rightArm_joint5', 'Baxter_rightArm_joint6', 'Baxter_rightArm_joint7'});
                                    
                try
                    obj.getJointHandlesFKarm({'Baxter_rightArm_joint1FK', 'Baxter_rightArm_joint2FK', 'Baxter_rightArm_joint3FK', 'Baxter_rightArm_joint4FK', ...
                                           'Baxter_rightArm_joint5FK', 'Baxter_rightArm_joint6FK', 'Baxter_rightArm_joint7FK'});
                catch
                    fprintf('\n\n *** this Baxter model does not have a FK implementation ***\n\n');
                end
                obj.nDoF = 7;
                
            case 'IRB140'
                obj.getJointHandles({'IRB140_joint1', 'IRB140_joint2', 'IRB140_joint3', 'IRB140_joint4', ...
                                            'IRB140_joint5', 'IRB140_joint6'});       
                obj.nDoF = 6;
            case 'MTB'
                obj.getJointHandles({'MTB_axis1', 'MTB_axis2', 'MTB_axis3', 'MTB_axis4'});       
                obj.nDoF = 4;                
        end
        
        
  
        
    end
    
    function [] = backToRestPosture(obj)

        % 1. first move the dummy target back to the rest. This does not guarantee that the arm will
        % actually achieve the rest posture
        xyz = obj.TrestPosture(1:3,4,1);
        rpy = tr2rpy(obj.TrestPosture);
        obj.sendTargetCartesianCoordinates(xyz, rpy, obj.entt.handle(strcmp('Dummy_target', obj.entt.name)), 0 );

        % 2. now set all joints to their rest posture
        obj.setJointAngles(obj.qRestPosture);
        if 0
            qrest = obj.qRestPosture;

            for j = 1:numel(qrest)
                %jointHandle = vrepm.objh.(['joint' num2str(j)]);
                jointHandle  = obj.joint.handle(j);
                if j~=numel(qrest)
                    obj.vrep.simxSetJointPosition(obj.clientID, jointHandle, qrest(j),  obj.vrep.simx_opmode_oneshot);
                else % for the last set command, use simx_opmode_oneshot_wait to force an update on the robot configuration
                    obj.vrep.simxSetJointPosition(obj.clientID, jointHandle, qrest(j),  obj.vrep.simx_opmode_oneshot_wait);
                end
            end
        end
    end
    
    function [] = setJointAngles(obj, q, FKMode)
        
        if isnumeric(q)
            for j = 1:numel(q)
                if exist('FKMode', 'var')
                    jointHandle  = obj.jointFK.handle(j);
                else
                    jointHandle  = obj.joint.handle(j);
                end
                if j~=numel(q)
                    obj.vrep.simxSetJointPosition(obj.clientID, jointHandle, q(j),  obj.vrep.simx_opmode_oneshot);
                else % for the last set command, use simx_opmode_oneshot_wait to force an update on the robot configuration
                    obj.vrep.simxSetJointPosition(obj.clientID, jointHandle, q(j),  obj.vrep.simx_opmode_oneshot_wait);
                end
            end
        else
            disp('!!skipping set joint angle because it is not a number (robot.setJointAngles)');
        end
        
    end
    
    
    function [T, xyz, rpy] = readEntityCoordinate(obj, name_)
        % Use this to get the positon of a specific element.

        index         = obj.entt.index(strcmp(obj.entt.name, name_));
        [xyz, rpy, T] = obj.readGenericCoordinates(index);

    end      


    function [xyz, rpy, To1, qrobot, qhuman] = readGenericCoordinates(obj, indexDummy)
    % function [xyz, rpy, To1, q] = readGenericCoordinates(obj, indexDummy)
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
            %qrobot = q(1:7);

            if nargout > 4
                qhuman = 'implementation needs improvement';
            end
        end
    end
    
    function [] = sendTargetCartesianCoordinates(obj, xyz, rpy, handleDummy, one_shot_wait_flag)
        % robot.sendTargetCartesianCoordinates(T(1:3,4,k), tr2rpy(T(:,:,k)), robot.getHandle('Dummy_target'), 1)
       
        % send position of target 
        obj.vrep.simxSetObjectPosition(obj.clientID, handleDummy, -1, xyz, obj.vrep.simx_opmode_oneshot);

        % send orientation of target    
        if one_shot_wait_flag
            obj.vrep.simxSetObjectOrientation(obj.clientID, handleDummy, -1, rpy, obj.vrep.simx_opmode_oneshot_wait);
        else
            obj.vrep.simxSetObjectOrientation(obj.clientID, handleDummy, -1, rpy, obj.vrep.simx_opmode_oneshot);            
        end
                         
    end
    

    function h = getHandle(obj, name)
        % 
        h =  obj.entt.handle(  strcmp(obj.entt.name, name)  );        
    end
    
    function h = getIndex(obj, name)
        h =  obj.entt.index(  strcmp(obj.entt.name, name)  );        
    end    
    
    % Fill the property obj.entt given the list of names in unsortedList
    function obj = getDummyHandlesFromVREP(obj, unsortedList)    

        vrep = obj.vrep;
        
        % get handles for dummies =====================================
        number_dataType = 0; % 0 is for object names
        [~,array_handles,~,~,array_stringData]= vrep.simxGetObjectGroupData(...
                                                 obj.clientID, vrep.sim_object_dummy_type,...
                                                 number_dataType, vrep.simx_opmode_oneshot_wait);
        % find the handle for the dummy target
        for k=1:numel(array_stringData(:,1))
            if sum(strcmp(array_stringData{k}, unsortedList))
                obj.entt.name   = [obj.entt.name   {array_stringData{k}}];
                obj.entt.index  = [obj.entt.index  k];
                obj.entt.handle = [obj.entt.handle array_handles(k)];
            end
        end
        if numel(unsortedList)~=numel(obj.entt.index)
            error('Some of the listed entities were not found in VREP scene.')
        end
    end
    
    function out = goTo(obj, Tstart, Tend, nTraj)
    % Linear interpolation of homog transf matrices
    % in XYZ with quaternion interpolation for the rotations
    %
        x_ = linspace(Tstart(1,4), Tend(1,4), nTraj);
        y_ = linspace(Tstart(2,4), Tend(2,4), nTraj);
        z_ = linspace(Tstart(3,4), Tend(3,4), nTraj);
        
        % buid the input homog. transf. matrices
        for k =1:nTraj
            in.T(:,:,k) = eye(4);
            in.T(1:3,4,k) = [x_(k) y_(k) z_(k)]';
        end
        
        poses(:,:,1) = Tstart;
        poses(:,:,2) = Tend;
        indexes = [1 nTraj];
        traj = interpolate_poses(in, indexes, poses);
        
        out = traj.T;
        
    end
    
    % Fill the property obj.entt given the list of names in unsortedList
    function obj = getJointHandles(obj, unsortedList)    
    
        vrep = obj.vrep;
        number_dataType = 0; % 0 is for object names
        [~,array_handles,~,~,array_stringData]= vrep.simxGetObjectGroupData(obj.clientID, vrep.sim_object_joint_type, ...  
                                                number_dataType, vrep.simx_opmode_oneshot_wait);

        % find the handle for the dummy target
        for k=1:numel(array_stringData(:,1))
            if sum(strcmp(array_stringData{k}, unsortedList))
                obj.joint.name   = [obj.joint.name   {array_stringData{k}}];
                obj.joint.index  = [obj.joint.index  k];
                obj.joint.handle = [obj.joint.handle array_handles(k)];
            end
        end
        if numel(unsortedList)~=numel(obj.joint.index)
            error('Some of the listed entities were not found in VREP scene.')
        end
        
        % check that indexes appear in the order 1 to 7. Otherwise
        % this may cause unexpected error, for example, when using
        % the method "move_arm_back_to_rest_posture(obj)"
        if sum(diff(diff(obj.joint.index)))~=0
            error('The joint handles are not ordered. This may cause unexpected behavior.');
        end

    end

    
    % Fill the property obj.entt given the list of names in unsortedList
    function obj = getJointHandlesFKarm(obj, unsortedList)    
    
        vrep = obj.vrep;
        number_dataType = 0; % 0 is for object names
        [~,array_handles,~,~,array_stringData]= vrep.simxGetObjectGroupData(obj.clientID, vrep.sim_object_joint_type, ...  
                                                number_dataType, vrep.simx_opmode_oneshot_wait);

        % find the handle for the dummy target
        for k=1:numel(array_stringData(:,1))
            if sum(strcmp(array_stringData{k}, unsortedList))
                obj.jointFK.name   = [obj.jointFK.name   {array_stringData{k}}];
                obj.jointFK.index  = [obj.jointFK.index  k];
                obj.jointFK.handle = [obj.jointFK.handle array_handles(k)];
            end
        end
        if numel(unsortedList)~=numel(obj.jointFK.index)
            error('Some of the listed entities were not found in VREP scene.')
        end
        
        % check that indexes appear in the order 1 to 7. Otherwise
        % this may cause unexpected error, for example, when using
        % the method "move_arm_back_to_rest_posture(obj)"
        if sum(diff(diff(obj.jointFK.index)))~=0
            error('The joint handles are not ordered. This may cause unexpected behavior.');
        end

    end
    
    
    
    function obj = requestStreaming( obj )
        % make one call for the joint streaming to make the server aware requests are coming
        % As far as I remember this makes the response from vrep faster
        vrep = obj.vrep;
        dataTypeJoint = 15; % 15: retrieves joint state data (in floatData (2 values): position, force/torque)
        disp('waiting for streaming accept.')  
        while vrep.simxGetObjectGroupData(obj.clientID, vrep.sim_object_joint_type, dataTypeJoint , vrep.simx_opmode_streaming) ~= 0
        end
        disp('Streaming mode set!')  
    end  
    
    function obj = simStart( obj )
        obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
        pause(0.1);
    end  
    function obj = simPause( obj )
        obj.vrep.simxPauseSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
    end  
    function obj = simStop( obj )
        obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot_wait);
        pause(0.1);
    end      
    
end
    
    

    
end
