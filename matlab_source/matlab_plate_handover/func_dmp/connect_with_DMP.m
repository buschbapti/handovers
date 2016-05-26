% 
% function solutionRobotFinal2 = connect_with_DMP(robot, qIdealStart, qIdealFinal, qOld )
% % 
% % solutionRobotFinal = 
% % 
% %             T: [4x4x124 double]
% %             p: [3x124 double]
% %         theta: 'roll_out_refFrame.m bug'
% %           eps: [0 0 0]
% %     theta_eps: [3x1 double]
% 
% 
%        
%     % Connect gaps with DMP
%     % Do an independent joint treatment    
%     
%     param.timeFactorForSteadyState = 1.5;
%     param.alphaBetaFactor          = 4;
%     param.debugFigures = [0 0];
%     for j= 1:numel(qOld(1,:))
%         
%         trajIn = qOld(:,j)';
%         
%         if ~isempty(qIdealStart)
%             param.xi = qIdealStart(j);
%         else
%             param.xi = trajIn(1);
%         end
%         if ~isempty(qIdealFinal)
%             param.xf = qIdealFinal(j);
%         else
%             param.xf = trajIn(end);
%         end
%        
%         param.minVel = 0.001;
%         
%         trajOut(:,j) = dmp(trajIn, param)';
%         
%     end
%     
%     
%     
%     
%     % Run simulation to get the forward kinematics solution
%     
%     % Stop simulation such that FK works
%     robot.simStop; 
%     disp('running IK');
%     indexDummy = robot.getIndex('Dummy_tip');
%     ctr=0;
%     for  t=1:numel(trajOut(:,1))
%         if ~mod(ctr,10)
%             fprintf('step %g of %g.\n', t, numel(trajOut(:,1)));
%         end
%         robot.setJointAngles(trajOut(t,:));
%        
%         % read again the tip, hopefully the solution makes sense
%         [~, ~, T(:,:,t)]    = robot.readGenericCoordinates(indexDummy);
%         ctr=ctr+1;
%     end
%     
%     solutionRobotFinal2.T = T;
%     solutionRobotFinal2.p = squeeze( T(1:3,4,:) );
%     solutionRobotFinal2.q = trajOut;
%     
%     robot.simStart;
%     
% end
% 
% 
