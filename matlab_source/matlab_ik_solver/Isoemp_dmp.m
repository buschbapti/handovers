classdef Isoemp_dmp < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        vp         = [];
        obst
        theta_mean
        theta_Cov
        theta_mean_pert
        
        theta_mean_Frame
        theta_Sigma_Frame
        theta_Cov_Frame
        
        theta_mean_dmpx 
        theta_mean_dmpy 
        theta_mean_dmpz 
        
        theta_Cov_dmpx 
        theta_Cov_dmpy 
        theta_Cov_dmpz 
        
        theta_mean_pert_Frame
        theta_mean_pert_dmpx
        theta_mean_pert_dmpy
        theta_mean_pert_dmpz
        
        refTraj
        refOrientation
        
        automatic_cov_update =0;
        s
        
        TrollOut
        refTrajPerturbed
        
        nTraj
        %adaptShapeFlag=1;
        
    end
    
    methods
        
        function obj = Isoemp_dmp(viaPoint, obstacle, nTraj)
           obj.vp = viaPoint;
           obj.obst = obstacle;

                %                        x         y        z      roll    pitch    yawl
            obj.theta_mean_Frame    = [  0         0        0        0       0       0                0         0       0              0   0       0 ];
            obj.theta_Sigma_Frame   = [  0.000   0.000  0.000     0*[  0.20     0.20      2 ]*pi/180     1*[0.0125 0.0125  0.0125]    0.5*[0.0125 0.0125  0.0125] ];
            obj.theta_Cov_Frame =  diag(obj.theta_Sigma_Frame);
            
            if mod(nTraj,10)
                error('To avoid conflict with the DMP code, use nTraj as a multiple of 10.')
            end
            obj.nTraj= nTraj;
            
            nTheta = obj.nTraj/10;
            
            obj.theta_mean_dmpx = [zeros(1,nTheta)];
            obj.theta_mean_dmpy = [zeros(1,nTheta)];
            obj.theta_mean_dmpz = [zeros(1,nTheta)];

            % create the covariance matrix to perturb the dmp parameters
            if 0 % old method is linear
                obj.theta_Cov_dmpx = diag(linspace(1,1000,nTheta));
                obj.theta_Cov_dmpy = diag(linspace(1,1000,nTheta));
                obj.theta_Cov_dmpz = diag(linspace(1,1000,nTheta));
            else % perturbe exponentially according to dmp phase Z
                Z = exp(-[1:(nTheta)]/((nTheta)*0.2 )) ;
                Z = 1./Z;
                Z = Z-Z(1)+1;
                amplit = 0.00*5000; % maximum amplitue of noise
                Z = amplit*Z/(Z(end)-Z(1));                
%                 figurew
%                 plot(linspace(1,1000,nTheta))
%                 plot(Z, 'r')
                obj.theta_Cov_dmpx = diag(Z);
                obj.theta_Cov_dmpy = diag(Z);
                obj.theta_Cov_dmpz = diag(Z);                
            end
        
        end

        function obj = initialGuessStart(obj, guessState)
          obj.theta_mean_Frame(7:9)= guessState';
        end
        
        function obj = initialGuessEnd(obj, guessState)
          obj.theta_mean_Frame(10:12)= guessState';
        end
        
        function obj = lockInitialState(obj)
            obj.theta_Sigma_Frame(7:9)  = 0;
            obj.theta_Cov_Frame =  diag(obj.theta_Sigma_Frame);
        end
        
        function obj = lockFinalState(obj)
          obj.theta_Sigma_Frame(10:12) = 0;
          obj.theta_Cov_Frame = diag(obj.theta_Sigma_Frame);
        end
        
        function obj = deactivateZsearch(obj)

          obj.theta_mean_Frame(3)= 0;
          obj.theta_mean_Frame(9)= 0;
          obj.theta_mean_Frame(12)= 0;
          
          obj.theta_Sigma_Frame(3)= 0;
          obj.theta_Sigma_Frame(9)= 0;
          obj.theta_Sigma_Frame(12)= 0;
          
          obj.theta_Cov_Frame = diag(obj.theta_Sigma_Frame);            
          obj.theta_Cov_dmpz= 0*obj.theta_Cov_dmpz;
         
        end
        
        function obj = getCovGain(obj, nUpdates, i)
            if obj.automatic_cov_update
                obj.s=1;
            else
                obj.s = (i-1)*(-1/(nUpdates-1)) + 1;
                obj.s = max(obj.s, 0.025); % keep a minimum of scale
            end              
        end
        
        function weps = sampleNoiseFrame(obj)
            weps  = mvnrnd(zeros(1,numel(obj.theta_mean_Frame)), obj.s*obj.theta_Cov_Frame);            
        end
        function weps = sampleNoiseDMP(obj, axis)
            switch axis
                case 'x'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_dmpx)), obj.s*obj.theta_Cov_dmpx);
                case 'y'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_dmpx)), obj.s*obj.theta_Cov_dmpy);
                case 'z'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_dmpx)), obj.s*obj.theta_Cov_dmpz);
            end
        end
        
         
        function h  = generate_trajectory(obj, h, j, flagClean, plotRollOuts, fixViaPointPoses, allowDMPShapeChange)
            
            thetaF = obj.theta_mean_pert_Frame(j,:)';
            
            oTa = rpy2tr(thetaF(4:6)') + [ zeros(4,3)   [thetaF(1:3); 0] ]; 

            for k = 1:numel(obj.refTraj(1,1,:))
                letterAr(:,:,k) = oTa*[ obj.refTraj(:,:,k) ];
            end
            
            if 1 % original formulation
                letterArXYZ = squeeze(letterAr(1:3,4,:));
            else
                letterArXYZ = squeeze(letterAr(1:3,4,:));
            end
            
            if allowDMPShapeChange % adapt shape by dmp                
              
                param.nTraj = obj.nTraj;
                param.alphaBetaFactor = 4;

                % regress DMP on transformed letter
                xdmp   = dmp_regression(letterArXYZ(1,:), param);
                xdmp.w = xdmp.w + obj.theta_mean_pert_dmpx(j,:)';

                ydmp = dmp_regression(letterArXYZ(2,:), param);
                ydmp.w = ydmp.w + obj.theta_mean_pert_dmpy(j,:)';

                zdmp = dmp_regression(letterArXYZ(3,:), param);
                zdmp.w = zdmp.w + obj.theta_mean_pert_dmpz(j,:)';

                paramgen.timeFactorForSteadyState = 1.0;
                t_normalized = linspace(0,1,numel(letterArXYZ(1,:)));
                
                % generalize
                paramgen.xi = thetaF(7);
                paramgen.xf = thetaF(10);
                x2    =  dmp_generalize(xdmp, paramgen);

                paramgen.xi = thetaF(8);    
                paramgen.xf = thetaF(11);
                y2    =  dmp_generalize(ydmp, paramgen);

                paramgen.xi = thetaF(9);    
                paramgen.xf = thetaF(12);
                z2    =  dmp_generalize(zdmp, paramgen);

                % include the final oscilating behavior in the trajectory
                if paramgen.timeFactorForSteadyState ~= 1
                    xyz2 = interp1(linspace(0,1,numel(z2)), [x2' y2' z2'], t_normalized );
                    x2 = xyz2(:,1)';
                    y2 = xyz2(:,2)';
                    z2 = xyz2(:,3)';                    
                end
                
                
                if fixViaPointPoses % keep rotations fixed according to refTraj
                    for k=1:numel(obj.refTraj(1,1,:))
                        Tnew(:,:,k) =  [ [obj.refOrientation(1:3,1:3,k); [0 0 0]]   [x2(k); y2(k); z2(k); 1]] ;
                    end
                else % use the new rotations along the trajectory
                    for k=1:numel(obj.refTraj(1,1,:))
                        Tnew(:,:,k) =  [ [letterAr(1:3,1:3,k); [0 0 0]]   [x2(k); y2(k); z2(k); 1]] ;
                    end                    
                end
                
            else % no need to do dmp                
                if fixViaPointPoses % keep rotations fixed according to refTraj
                    for k=1:numel(obj.refTraj(1,1,:))
                        Tnew(:,:,k) =  [ [obj.refTraj(1:3,1:3,k); [0 0 0]]   [letterArXYZ(:,k); 1]] ;
                    end
                else % use the new rotations along the trajectory
                    Tnew = letterAr;                   
                end                
            end
                        
            obj.TrollOut(:,:,:,j) = Tnew;
            obj.refTrajPerturbed(:,:,:,j) = letterAr;   
            
            %% plot trajectory
            
            xyzNew = squeeze(Tnew(1:3,4,:));
            dbg = 1;
            if dbg
                figure(h.fig)
                if flagClean ~= 0
                   if plotRollOuts
                        h.plotCart = [h.plotCart plot3(xyzNew(1,:), xyzNew(2,:), xyzNew(3,:), sty(lightRGB(1),[],1))];
                   end
                else
                    if 1 % this plots the reference trajectory
                        param.hfig = h.fig;
                        param.axis_length = 0.03;
                        param.nMaxPlots = 5;
                        param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');
                        param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 .5 .5]);
                        param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.5 1 .5]);
                        param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.5 .5 1]);
                        param.shadowHeight     = -0.475;
                        [~, htmp] = homogTransfPlot(Tnew, param);
                        h.plotCart = [h.plotCart htmp];
                    end
                    h.plotCart = [h.plotCart plot3(xyzNew(1,:), xyzNew(2,:), xyzNew(3,:), sty('b',[],2))];
                end
            end
            
            
        end

        function obj = warpTheta(obj)
            obj.theta_mean      = [obj.theta_mean_Frame'; obj.theta_mean_dmpx'; obj.theta_mean_dmpy'; obj.theta_mean_dmpz']';
            obj.theta_Cov       = blkdiag(obj.theta_Cov_Frame, obj.theta_Cov_dmpx, obj.theta_Cov_dmpy, obj.theta_Cov_dmpz);
            obj.theta_mean_pert = [obj.theta_mean_pert_Frame obj.theta_mean_pert_dmpx obj.theta_mean_pert_dmpy obj.theta_mean_pert_dmpz];
        end

        function obj = unwarpTheta(obj, newThetaMean)
            
            ir = 1:numel(obj.theta_mean_Frame);
            obj.theta_mean_Frame = newThetaMean(ir);
            
            irx = ir(end)+1:ir(end)+numel(obj.theta_mean_dmpx);
            obj.theta_mean_dmpx = newThetaMean(irx);
            
            iry = irx(end)+1:irx(end)+numel(obj.theta_mean_dmpy);
            obj.theta_mean_dmpy = newThetaMean(iry);
            
            irz = iry(end)+1:iry(end)+numel(obj.theta_mean_dmpz);
            obj.theta_mean_dmpz = newThetaMean(irz);
            
        end
        
        function updateCME(obj)
            obj.warpTheta;
            [theta_mean_new] = updateCME(obj.theta_mean, obj.theta_Cov, obj.theta_mean_pert, obj.cost_total, 'Pi2');
            obj.unwarpTheta(theta_mean_new); 
        end

        function restart_main(obj)
            
            obj.theta_mean_pert =[];
            
            obj.TrollOut =[];
            obj.refTrajPerturbed =[];

            obj.theta_mean_pert =[];
            obj.theta_mean_pert_Frame=[];
            obj.theta_mean_pert_dmpx =[];
            obj.theta_mean_pert_dmpy =[];
            obj.theta_mean_pert_dmpz =[];
            
     
            
        end        

        
    end
    
end






