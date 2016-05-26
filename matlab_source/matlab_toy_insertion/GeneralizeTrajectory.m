classdef GeneralizeTrajectory < handle
    
    properties
        grid
        soundPlayer
        qTraj1
        param
        hfig
    end
    
    methods
        
        function obj = GeneralizeTrajectory( param )            
            obj.soundPlayer = preloadSound_Baxter;
            obj.param = param;
            
            % load grid
            switch param.grid_type
                case 'relative'
                    load('./data/fine_grid_relative.mat');
                case 'reba'
                    load('./data/fine_grid_reba.mat');
            end                    
            obj.grid = solGrid;
            
            % compute quaternions at the end of the trajectory to make lookup table faster            
            for k =1:numel(obj.grid)
                quat_ = Quaternion(obj.grid{k}.T(:,:,end));
                obj.grid{k}.quaternionEnd = [quat_(end).s  quat_(end).v];
            end            
        end
        
        function makeRobotTrajectory(obj, posesMatlabFormat, robot)            
            if obj.param.generalizeMethod == 1 % use DMP
                obj.makeRobotTrajectoryGeneralize(posesMatlabFormat, robot);
            end
            if obj.param.generalizeMethod == 2 % use NN
                obj.makeRobotTrajectoryNN(posesMatlabFormat, robot);
            end
            
            % smooth trajectory here
           % figurew('jointTraj');
            % plot(obj.qTraj1, 'LineWidth', 2, 'LineStyle', '--');

            if 1
                paramFilter.filterOrder = 3; 
                paramFilter.filterFreq  = 0.05; % normalized freq [ 1]
                for j=1:7
                    q(:,j) = obj.smoothf(obj.qTraj1(:,j), paramFilter);
                end
                %plot(q, 'LineWidth', 2);
                obj.qTraj1 = q;
            end
            
            
            if obj.param.speedUpWithoutFKanimation ~=1
                for t =1:numel(obj.qTraj1(:,1))
                    robot.setJointAngles(obj.qTraj1(t,:),1);
                end
            else
                robot.setJointAngles(obj.qTraj1(end,:),1);
                %pause(2)
            end
        end
                
        function makeRobotTrajectoryGeneralize(obj, posesMatlabFormat, robot)         
            [ initGuess, Ttarget ] = obj.findClosestElementOnGrid(  posesMatlabFormat(2,:) );            
            robot.setCartesian(Ttarget, 'Dummy_target');
            robot.setCartesian(Ttarget, 'handoverPosition');

            % check which joint guess gives better results
            robot.setJointAngles(  initGuess.q(1,:)  ); pause(1);
            TtipA    = robot.readEntityCoordinate('Dummy_tip');
            qTargetA = robot.getJointAngles();
            robot.setJointAngles(  initGuess.q(end,:)  );  pause(1);
            TtipB = robot.readEntityCoordinate('Dummy_tip');
            qTargetB = robot.getJointAngles();                
            
            if ~isempty(obj.hfig) % check that the robot can achieve this position
                param.hfig = obj.hfig;
                param.axis_length = 0.35;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', [0.75 0 0 ]);
                param.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', [0 0.7  0 ]);
                param.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', [0 0 0.75 ]);
                homogTransfPlot(TtipA, param);
                param.axis_length = 0.25;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 8, 'Color', [0.75 0 0 ]);
                param.axesPlotStyle{2} = struct('LineWidth', 8, 'Color', [0 0.7  0 ]);
                param.axesPlotStyle{3} = struct('LineWidth', 8, 'Color', [0 0 0.75 ]);                
                homogTransfPlot(TtipB, param);
            end
            
            qtipA_ = Quaternion(TtipA);
            [dxyzA, dquatA] = obj.xyzQuaternionDistance( posesMatlabFormat(2,:), [TtipA(1:3,4)' qtipA_ .s qtipA_ .v]  );

            qtipB_ = Quaternion(TtipB);
            [dxyzB, dquatB] = obj.xyzQuaternionDistance( posesMatlabFormat(2,:), [TtipB(1:3,4)' qtipB_ .s qtipB_ .v]  );
            
            [dxyzNN, dquatNN] = obj.xyzQuaternionDistance( posesMatlabFormat(2,:), initGuess.quaternionEnd  );

            wDist = 0.1;
            d = wDist*[dxyzA dxyzB dxyzNN]./sum([dxyzA dxyzB dxyzNN]) +...
                (1-wDist)*[dquatA dquatB dquatNN ]./sum([dquatA dquatB dquatNN]);

            [~, idx] = min(d);        
            if idx==3
                keyboard 
            end
            robot.backToRestPosture;
            % pick the type with lower error
            if idx == 1 
                qTarget = qTargetA;
                fprintf('Generalize DMP. Init Guess type 1.\n');
            end
            if idx == 2
                qTarget = qTargetB;
                fprintf('Generalize DMP. Init Guess type 2.\n');
            end            
            if idx==1 || idx==2                      
                paramDMP.Dgain = 300;
                paramDMP.nBasis = 30;
                paramDMP.forcingFunctionScaling=1;
                % Generalize DMPs in joint space using qTarget
                for j=1:7
                    dmpJoint{j} = Dmp(initGuess.q(:,j)', 'hoffmann', paramDMP);
                    param.xf = qTarget(j);
                    qNew{j}  = dmpJoint{j}.generalize(param)';
                end
                obj.qTraj1 = obj.sync_dmps(qNew);  
            end
            
            if idx==3 % if NN no need to generalize DMP
                obj.qTraj1 = initGuess.q;
                fprintf('Using NN solution.\n');
            end

            
        end
        
        function makeRobotTrajectoryNN(obj, posesMatlabFormat, robot)         
            [ initGuess ] = obj.findClosestElementOnGrid(  posesMatlabFormat(2,:) );
            Ttarget = initGuess.T(:,:,end);
            robot.setCartesian(Ttarget, 'Dummy_target');
            robot.setJointAngles(initGuess.q(end,:));
            pause(1.0)
            dbg = 1; 
            if dbg % check that the robot can achieve this position
                Ttip = robot.readEntityCoordinate('Dummy_tip');
                param.hfig = gcf;
                param.axis_length = 0.25;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 8, 'Color', [0.75 0 0 ]);
                param.axesPlotStyle{2} = struct('LineWidth', 8, 'Color', [0 0.7  0 ]);
                param.axesPlotStyle{3} = struct('LineWidth', 8, 'Color', [0 0 0.75 ]);
                homogTransfPlot(Ttip, param); 
            end        
            obj.qTraj1 = initGuess.q;            
        end
        
        
        
        function [initGuess, T] = findClosestElementOnGrid( obj, xyzquat )
            
            T = quaternion2homogTransfMatrix( xyzquat(4:end) );
            T(1:3,4) = xyzquat(1:3)';
            
            xyz  = xyzquat(1:3);
            quat = xyzquat(4:end);
            for t = 1:numel(obj.grid)                
                dxyz(t) = sqrt(sum((xyz'-obj.grid{t}.T(1:3,4, end)).^2));                
                normQuat = obj.grid{t}.quaternionEnd./norm(obj.grid{t}.quaternionEnd);                
                dquat(t) = acos( 2*sum(quat.*normQuat).^2-1);
                %dquat(t) = acos(sum(quat.*normQuat)); %Rudi's approach
                if dquat(t) > pi
                    error('Quat distance wrong??')
                end                 
            end

            % total distance is the normalized values of angle and
            % Cartesian coordinates.
            dtotal  = dquat./sum(dquat) + dxyz./sum(dxyz);           
            
            [~,idx] = min(dtotal);
            
            
            if obj.param.plotGrid
                param.hfig = cartesian_plot('checkGrid');
                title('DarkRGB: Target, lightRGB: NN');
                param.axis_length = 0.5;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0]);
                param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [0 1 0]);
                param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [0 0 1]);
                homogTransfPlot(T, param);
                
                param.axis_length = 0.25;
                param.axesPlotStyle{1} = struct('LineWidth', 0.4, 'Color', lightRGB(1));
                param.axesPlotStyle{2} = struct('LineWidth', 0.4, 'Color', lightRGB(2));
                param.axesPlotStyle{3} = struct('LineWidth', 0.4, 'Color', lightRGB(3));                
                for k=1:5:numel(obj.grid)
                    homogTransfPlot(obj.grid{k}.T(:,:,end), param);
                end
            
                param.axis_length = 0.5;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', lightRGB(1));
                param.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', lightRGB(2));
                param.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', lightRGB(3));
                homogTransfPlot(obj.grid{idx}.T(:,:,end), param);
                
                obj.hfig = param.hfig;
            end
            
           initGuess = obj.grid{idx};             
        end
        
        function [] = write_trajectory_file(obj, storePath)
        % write_trajectory_file(storePath, traj1_, traj2_, nTraj, initDT)
     
            tNow   = linspace(0, obj.param.tFinal, numel(obj.qTraj1(:,1)));           
            tDesired = linspace(0, obj.param.tFinal, obj.param.nTraj  );            
            desiredQ = interp1(tNow, obj.qTraj1, tDesired);
    
            tDesired = tDesired + obj.param.initialDT;    
            traj1 = [tDesired' desiredQ];

            try
                save([storePath '/traj1.txt'], 'traj1',  '-ascii', '-tabs');
            catch
                warning('ROS folder structure not found.');
            end

            try
                traj2 = [];
                save([storePath '/traj2.txt'], 'traj2',  '-ascii', '-tabs');
            catch
                warning('ROS folder structure not found.');
            end 
            
            try
                empty_content = [];
                save([storePath '/flagMatlabFinished.txt'], 'empty_content',  '-ascii', '-tabs');
            catch
                warning('ROS folder structure not found.');
            end 

        end
        
        function []=resetForNexIteration(obj)
            obj.qTraj1 = [];
        end
        
    
        function x  = forceHumanAwayFromRobot(obj, x)
            
            % force human to be far from robot or IK will fail
            if x < obj.param.minMaxDistFromRobot(1)
                x = obj.param.minMaxDistFromRobot(1);
            end
            if x > obj.param.minMaxDistFromRobot(2)
                x = obj.param.minMaxDistFromRobot(2);
            end               
            
        end        
        
    end % methods
    
    methods (Static)
        
        function sceneBackToOriginalPosition(robot) 
            T_ = eye(4); T_(1:3,4) = [1  0.5 -1.22];
            robot.setCartesian(T_, 'Dummy_viaPoint_table');
            robot.setCartesian(T_, 'handoverPosition');
            robot.setCartesian(T_, 'Dummy_tip_humanA_L');
            robot.setCartesian(T_, 'Dummy_tip_humanA_R');
            robot.backToRestPosture;
        end
        
        function posesFromROS = createFakePose(noiseDeg, noiseCart, rotIndex, xyz)
            % posesFromROS = createFakePose( type, entryNumber, rotationNumber, noiseDeg, noiseCart) 
            %   xyz: this is optional if you want to force a specific
            %   position
            %   rotIndex: rotation of end effector (5 possible)
            %   noiseDeg: stdev of noise in degrees
            %   noiseCart: stdev of noise in meters
            
            if isempty(xyz)
                xyz = [0.5  0  -0.27];
            else
                noiseCart = 0; % force the xyz to be exact
            end
           
            T = se3([0 pi/2 0  0 0 0]);
            T = se3([pi 0 0  0 0 0])*T;
            T(1:3,4) = xyz';
            
            angleStep = d2r(360/5);
            
            noiseAngle = d2r(noiseDeg)*randn(1,3);
            T_ = T*se3([ noiseAngle(1)  noiseAngle(2) noiseAngle(3)+(rotIndex-1)*angleStep  0  0 0]);

            noiseXYZ = noiseCart*randn(1,3)';
            T_(1:3,4) = [noiseXYZ(1)+xyz(1)  noiseXYZ(2)+xyz(2)  noiseXYZ(3)+xyz(3)];
                
            posesFromROS = zeros(3, 7);
            quat_ = Quaternion(T_);
            posesFromROS(2,:) = [T_(1:3,4)' [quat_.v quat_.s]];
            
        end
        
        
        
        function posesFromROS = createFakePose_DEPRECATED( type, entryNumber, rotationNumber, noiseDeg, noiseCart) 
            % posesFromROS = createFakePose( type, entryNumber, rotationNumber, noiseDeg, noiseCart) 
            %   type 'grid_reba'  'grid_relative'
            %   entryNumber: which in the original grid of Baptiste should we get the position
            %   rotationNumber: rotation of end effector (5 possible)
            %   noiseDeg: stdev of noise in degrees
            %   noiseCart: stdev of noise in meters
            
            posesFromROS = zeros(3, 7);
            out = grid_endeffector_positions_insertion_toy( type, entryNumber );            
            
            angleStep = d2r(360/5);
            
            dbg = 0;
            if dbg
                param.hfig = cartesian_plot('df');
                param.axis_length = 0.5;
                param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
                param.axesPlotStyle{1} = struct('LineWidth', 1.1, 'Color', [1 .5 .5]);
                param.axesPlotStyle{2} = struct('LineWidth', 1.1, 'Color', [.5 1 .5]);
                param.axesPlotStyle{3} = struct('LineWidth', 1.1, 'Color', [.5 .5 1]);
                homogTransfPlot(out.Tinria, param);

                param.axis_length = 0.3;
                param.axesPlotStyle{1} = struct('LineWidth', 4.1, 'Color', [1 0 0 ]);
                param.axesPlotStyle{2} = struct('LineWidth', 4.1, 'Color', [0 1 0]);
                param.axesPlotStyle{3} = struct('LineWidth', 4.1, 'Color', [0 0  1]);            
                for k=1:5
                    T_ = out.Tinria*se3([0 0 (k-1)*angleStep  0 0 0])
                    homogTransfPlot(T_, param);
                end
            end
            
            rpyNoise = d2r(noiseDeg)*randn(1,3);
            xyzNoise = noiseCart*randn(1,3);
            
            T_ = out.Tinria*se3([rpyNoise(1)  rpyNoise(2)  rpyNoise(3)+(rotationNumber-1)*angleStep  xyzNoise]);
            quat_ = Quaternion(T_);
            posesFromROS(2,:) = [T_(1:3,4)' [quat_.v quat_.s]];

            
            
        end
        
        function vecOut = changeQuaternionOrder(vecIn)
            % ROS    uses   quat.x     quat.y quat.z quat.scale
            % Matlab uses   quat.scale quat.x quat.y quat.z 
            %
            % vecIn: ROS format
            % vecOut: Matlab format
            vecOut = vecIn;

            idx = [1 2 3 7 4 5 6];
            for k=1:numel(vecIn(:,1))
                vecOut(k,:) = vecIn(k,idx);
            end
        end
        
       
        function q = sync_dmps(qnewTraj)

            for j=1:numel(qnewTraj)
                ind(j) = numel(qnewTraj{j});
            end

            indMax = max(ind);
            for j=1:numel(qnewTraj)
                if numel(qnewTraj{j})~= indMax
                    q(:,j) = [qnewTraj{j}; qnewTraj{j}(end).*ones(indMax-numel(qnewTraj{j}-1),1)];
                else
                    q(:,j) = [qnewTraj{j}];
                end
            end 
        end
        
        function y = smoothf(yin, paramFilter)
        % This function replaces the "smooth" function of Matlab which is part of
        % the curve fitting toolbox and sometimes not found in many installations.
        % This implementation relies on the signal processing toolbox, which most
        % people seem to have.
        % You must set the filter order and frequency using paramFilter.
        %
        %
        % Example:
        %
        %     t = 0:0.01:2;
        %     y = zeros(size(t));
        %     y(50:end) = 1 + 0.05*randn( size(y(50:end)));
        % 
        %     figure; hold on;
        %     plot(t, y); 
        %     plot(t, smoothf(y), 'r')
        %
        %
        %
        %

            % set filter parameters
            if ~exist('paramFilter')
                paramFilter.filterOrder = 3; 
                paramFilter.filterFreq = 0.2; % normalized freq [ 1]
            end


            % construct the filter
            [b,a] = butter( paramFilter.filterOrder, paramFilter.filterFreq,  'low'); % IIR filter design

            % filter using non-causal method
            y = filtfilt(b,a, yin);    

        end
        
        function [dxyz, dquat, dtotal] = xyzQuaternionDistance( xyzquatA, xyzquatB )
            
            dxyz   = sqrt(  sum(    (xyzquatA(1:3)-xyzquatB(1:3)).^2   )    );
            dquat  = acos( 2*sum(xyzquatA(4:end).*xyzquatB(4:end)).^2-1);
            
            dtotal = dxyz + dquat;
            
        end

    end % methods Static
    
end



















