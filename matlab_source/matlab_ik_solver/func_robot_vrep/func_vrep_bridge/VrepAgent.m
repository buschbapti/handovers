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
        ikine_mask = [1 1 1  1 1 1];
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
        
        
    end   
    
    
    function [ik_hist] =  IK(robot, T, pause_ )
    %     ne: [1x33 double]
    %      T: [4x4x33 double]
    %      q: [33x3 double]
    %
    %   pause_: if empty: stop and wait for keyboard input
    %           if 0: do not pause at all
    %           if not zero, use the value as pause duration (in seconds)
    %

        % storage variable
        st.target.xyz = [];
        st.target.rpy = [];

        st.actual.xyz = [];
        st.actual.rpy = [];
        st.actual.q   = [];

        st.initTarget.xyz = [];
        st.initTarget.rpy = [];

        mask = robot.ikine_mask;

        nT = size(T,3);

        % independent of the trajectory, send it to the rest posture before starting any loop
        % The reasoning is that the location of the trajectory will favor being close to this posture
        robot.backToRestPosture( ); 

        handleDummyTarget  = robot.getHandle('Dummy_target');
        indexDummyTip      = robot.getIndex('Dummy_tip');

        for t = 1:nT
            % send target position
            xyz_ = T(1:3,4,t);  rpy_ = tr2rpy(T(:,:,t));        
            robot.sendTargetCartesianCoordinates(xyz_, rpy_, handleDummyTarget, 1);
            st.target.xyz = [ st.target.xyz;  xyz_' ];
            st.target.rpy = [ st.target.rpy;  rpy_' ];

            % get current tip position
            [xyz_, rpy_, Tcurr, qrobot_] = robot.readGenericCoordinates(indexDummyTip);
            st.actual.xyz = [st.actual.xyz; xyz_];
            st.actual.rpy = [st.actual.rpy; rpy_]; 
            st.actual.q   = [st.actual.q;   qrobot_];        

            e = tr2delta( Tcurr, T(:,:,t) );

            ik_hist.ne(:,t)  = norm(e (mask) );
            ik_hist.T(:,:,t) = Tcurr;

            if ~isempty(pause_)
                if pause_~=0
                    pause(pause_);
                end
            else
                pause;
            end

        end

        ik_hist.q = st.actual.q;    

        startIndex = 1;
        ik_hist.ne = ik_hist.ne(startIndex:end);
        ik_hist.T  = ik_hist.T(:,:,startIndex:end);
        ik_hist.q  = ik_hist.q(startIndex:end,:);    

    end

    
    function [] = backToRestPosture(obj)

        % 1. first move the dummy target back to the rest. This does not guarantee that the arm will
        % actually achieve the rest posture
        xyz = obj.TrestPosture(1:3,4,1);
        rpy = tr2rpy(obj.TrestPosture);
        obj.sendTargetCartesianCoordinates(xyz, rpy, obj.entt.handle(strcmp('Dummy_target', obj.entt.name)), 0 );

        % 2. now set all joints to their rest posture
        obj.setJointAngles(obj.qRestPosture);
    end
        
    
    function [] = setJointAngles(obj, q, FKMode)
        % so far only tested in Baxter
        
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
        h =  obj.entt.handle(  strcmp(obj.entt.name, name)  );        
    end
    
    function h = getIndex(obj, name)
        h =  obj.entt.index(  strcmp(obj.entt.name, name)  );        
    end    
    
    function obj = getDummyHandlesFromVREP(obj, unsortedList)    
    % Fill the property obj.entt given the list of names in unsortedList
    
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
    
    
    function out = goTo(obj, Tstart, Tend, nTraj)
    % Linear interpolation of homog transf matrices
    % in XYZ with quaternion interpolation for the rotations
    %
    
        if 0
            x_ = linspace(Tstart(1,4), Tend(1,4), nTraj);
            y_ = linspace(Tstart(2,4), Tend(2,4), nTraj);
            z_ = linspace(Tstart(3,4), Tend(3,4), nTraj);
        else
            x_ = obj.goToJointTrapez(Tstart(1,4), Tend(1,4), nTraj, []);
            y_ = obj.goToJointTrapez(Tstart(2,4), Tend(2,4), nTraj, []);
            z_ = obj.goToJointTrapez(Tstart(3,4), Tend(3,4), nTraj, []);
        end
        
        % buid the input homog. transf. matrices
        for k =1:nTraj
            in.T(:,:,k) = eye(4);
            in.T(1:3,4,k) = [x_(k) y_(k) z_(k)]';
        end
        
        poses(:,:,1) = Tstart;
        poses(:,:,2) = Tend;
        indexes = [1 nTraj];
        traj = interpolate_poses(in.T, indexes, poses);
        
        out = traj;        
    end
    
    
end



    
methods (Static)


    
    function yf  = goToJointTrapez(xi, xf, nSteps, rates)
        
        if isempty(rates)
            rates = 0.25;
        end

        nStart = round(nSteps*rates(1));
        yd1 = linspace(0, 1, nStart);
        yd3 = yd1(end:-1:1);
        yd2 = linspace(1,1,nSteps-2*(nStart));

        yd = [yd1 yd2 yd3];

        y = cumsum(yd);
        y = y/max(y);

        amp = xf-xi;
        yf = (xi+y*amp)';

        dbg=0;
        if dbg
            figurew('vel profile');
            plot(yd);
            plot(y, 'r');

            figurew('final sol');
            plot(yf, sty('b', 'o', 2));
        end   
 
    end
    
    
end
    
    

    
end
