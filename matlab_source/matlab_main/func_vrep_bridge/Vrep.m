classdef Vrep < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
       
    properties
        vrep      = [];
        clientID  = [];
        robotName = [];
        opt; % not sure if I still need this thing.
    end

    methods

        function obj = Vrep( )

            % initialization procedure
            vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections

            obj.vrep = vrep;

            fprintf('\n\n>Waiting for vrep\n');
            vrep_ready = -1;
            while vrep_ready ~= 0
                obj.clientID = obj.vrep.simxStart('127.0.0.1',19997,true,true,500,5);
                vrep_ready   = obj.clientID;
                fprintf('.');
            end
            fprintf('VREP ready!!!\n');

            obj.get_robot_name();
        
            obj.opt.fastloop   = 1; % do pseudo-batch process
            obj.opt.waitServer = 1;
            obj.opt.plotPause  = 0.1; % 0: no pause, x: pause for x sec, []: pause and wait for manual release.
                    
        end

        function obj = get_robot_name(obj)
            % function robot_name = get_robot_name(vrepDummyNames)
            %
            % This is an ugly hack that will get the names of the dummies in the scene.
            % It will then compare the dummy name with the possible robot names.
            % Given the order which the names are in "possible_robot_names" I hand code
            % the switch part which gives the robot name.
            % Everything here is hand coded and very ugly.
            %

            vrep = obj.vrep;
            number_dataType = 0;
            [~,~,~,~,vrepDummyNames]=...
                vrep.simxGetObjectGroupData(obj.clientID, vrep.sim_object_dummy_type,...
                                   number_dataType, vrep.simx_opmode_oneshot_wait);

            % names hand coded in vrep
            possible_robot_names = ({'floor_UR5', 'floor_LBR4', 'floor_Baxter'}); 

            nDummyNames = numel(vrepDummyNames(:,1));

            selected = -99 ;
            for k=1:nDummyNames
                nRobots = numel(possible_robot_names);
                for j=1:nRobots
                    if strcmp( vrepDummyNames{k}, possible_robot_names{j} )
                        selected = j;
                    end
                end
            end

            % This is also hand coded.
            switch selected
               case 1
                   obj.robotName = 'UR5';
               case 2
                   obj.robotName = 'LBR4';
               case 3
                   obj.robotName = 'Baxter';  
               otherwise
                   error('no known robot was identified. Did you load the appropriate scene in VREP? Did you start the simulation in VREP?');
            end

        end

        function [] = printStuff(obj)
            
            keyboard
            
        end
        



    end

    

    
end

