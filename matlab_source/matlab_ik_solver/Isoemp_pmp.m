classdef Isoemp_pmp < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        
        vp
        obst
        Gn
        
        XYZrollOut
        refTrajPerturbed
        
        theta_mean_Frame
        theta_Sigma_Frame
        theta_Cov_Frame
        
        theta_mean_pmpx
        theta_mean_pmpy
        theta_mean_pmpz
        
        theta_Cov_pmpx
        theta_Cov_pmpy
        theta_Cov_pmpz
        
        refTraj
        automatic_cov_update 
        s

        theta_mean_pert_Frame
        theta_mean_pert_pmpx
        theta_mean_pert_pmpy
        theta_mean_pert_pmpz
        
        
        theta_mean  
        theta_Cov   
        theta_mean_pert
                
            
    end
    
    methods
        
        function obj = Isoemp_pmp(xyzTraj, viaPoint, obstacle, opt)            
            
            if exist('opt', 'var')
                if isfield(opt, 'perturbInitStates')
                    oTa = rpy2tr(opt.perturbInitStates(4:6)) + [ zeros(4,3)   [opt.perturbInitStates(1:3)'; 0] ];                     
                    xyzTraj = oTa*[ xyzTraj; [ones(1,numel(xyzTraj(1,:)) )] ];
                    xyzTraj(4,:) = [];
                end
            end

            % get a ProMP here
            [obj.theta_mean_pmpx, obj.Gn] = rbf_regression(xyzTraj(1,:), 20);
            [obj.theta_mean_pmpy]         = rbf_regression(xyzTraj(2,:), 20);
            [obj.theta_mean_pmpz]         = rbf_regression(xyzTraj(3,:), 20);
            obj.theta_mean_pmpx = obj.theta_mean_pmpx';
            obj.theta_mean_pmpy = obj.theta_mean_pmpy';
            obj.theta_mean_pmpz = obj.theta_mean_pmpz';

            obj.vp   = viaPoint;
            obj.obst = obstacle;

            %                             x         y        z      roll    pitch    yawl
            obj.theta_mean_Frame    = [  0         0        0        0       0       0             ];
            obj.theta_Sigma_Frame   = [  0.0002   0.0002  0.0002   [ 0       0      1 ]*pi/180    ];


            obj.theta_Cov_pmpx = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpx), [5 5]);
            obj.theta_Cov_pmpy = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpy), [5 5]);
            obj.theta_Cov_pmpz = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpz), [5 5]);

            obj.consolidateTheta();
        end

%         function obj = initialGuessStart(obj, guessState)
%           obj.theta_mean(7:9)= guessState';
%         end
%         
%         function obj = initialGuessEnd(obj, guessState)
%           obj.theta_mean(10:12)= guessState';
%         end
%         
%         function obj = lockInitialState(obj)
%           obj.theta_Sigma(7:9)  = 0;
%         end
%         
%         function obj = lockFinalState(obj)
%           obj.theta_Sigma(10:12) = 0;
%         end
        
        function obj = deactivateZsearch(obj)

          obj.theta_mean_Frame(3)= 0;
          obj.theta_Sigma_Frame(3)= 0;
          obj.theta_Cov_pmpz = 0.*obj.theta_Cov_pmpz;
          
          obj.theta_mean_pmpz = 0*obj.theta_mean_pmpz;
            
          obj.consolidateTheta();
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
        function weps = sampleNoisePMP(obj, axis)
            switch axis
                case 'x'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_pmpx)), obj.s*obj.theta_Cov_pmpx);
                case 'y'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_pmpx)), obj.s*obj.theta_Cov_pmpy);
                case 'z'
                    weps  = mvnrnd(zeros(1,numel(obj.theta_mean_pmpx)), obj.s*obj.theta_Cov_pmpz);
            end
        end
        
        
        function obj = consolidateTheta(obj)
            obj.theta_Cov_Frame = diag(obj.theta_Sigma_Frame);
        end
        
        function h = generate_trajectory(obj, h, j, flagClean, plotRollOuts)
            
            thetaF = obj.theta_mean_pert_Frame(j,:)';
            
            oTa = rpy2tr(thetaF(4:6)') + [ zeros(4,3)   [thetaF(1:3); 0] ]; 
            letterAr = oTa*[ obj.refTraj; [ones(1,numel(obj.refTraj(1,:)) )] ];
            letterAr(4,:) = [];

            %plot(letterAr(1,:), letterAr(2,:), 'g')
            
            % recover
            x  = obj.Gn*obj.theta_mean_pert_pmpx(j,:)';
            y  = obj.Gn*obj.theta_mean_pert_pmpy(j,:)';
            z  = obj.Gn*obj.theta_mean_pert_pmpz(j,:)';

            
            xyzNew = oTa*[ [x'; y'; z']; [ones(1,numel(x) )] ];
            xyzNew(4,:) = [];
    
            % plot3(x, y, z)
            
            obj.XYZrollOut(:,:,:,j) = xyzNew;
            obj.refTrajPerturbed(:,:,:,j) = letterAr;           
            
            %% plot trajectory
            
            dbg = 1;
            if dbg
                figure(h.fig)
                if flagClean ~= 0
                   if plotRollOuts
                        h.plot = [h.plot plot3(xyzNew(1,:), xyzNew(2,:), xyzNew(3,:), sty(lightRGB(3),[],1))];
                   end
                else
                    h.plot = [h.plot plot3(xyzNew(1,:), xyzNew(2,:), xyzNew(3,:), sty('b',[],2))];
                end
            end
            
            
            
        end      

        function obj = warpTheta(obj)
            obj.theta_mean      = [obj.theta_mean_Frame'; obj.theta_mean_pmpx'; obj.theta_mean_pmpy'; obj.theta_mean_pmpz']';
            obj.theta_Cov       = blkdiag(obj.theta_Cov_Frame, obj.theta_Cov_pmpx, obj.theta_Cov_pmpy, obj.theta_Cov_pmpz);
            obj.theta_mean_pert = [obj.theta_mean_pert_Frame obj.theta_mean_pert_pmpx obj.theta_mean_pert_pmpy obj.theta_mean_pert_pmpz];
        end

        function obj = unwarpTheta(obj, newThetaMean)
            
%             keyboard
            ir = 1:numel(obj.theta_mean_Frame);
            obj.theta_mean_Frame = newThetaMean(ir);
            
            irx = ir(end)+1:ir(end)+numel(obj.theta_mean_pmpx);
            obj.theta_mean_pmpx = newThetaMean(irx);
            
            iry = irx(end)+1:irx(end)+numel(obj.theta_mean_pmpy);
            obj.theta_mean_pmpy = newThetaMean(iry);
            
            irz = iry(end)+1:iry(end)+numel(obj.theta_mean_pmpz);
            obj.theta_mean_pmpz = newThetaMean(irz);
            
        end
        
        function updateCME(obj)
            obj.warpTheta;
            [theta_mean_new] = updateCME(obj.theta_mean, obj.theta_Cov, obj.theta_mean_pert, obj.cost_total, 'Pi2');
            obj.unwarpTheta(theta_mean_new);            
        end

        function restart_main(obj)
            %                             x         y        z      roll    pitch    yawl
            obj.theta_Sigma_Frame   = [  0.0002   0.0002  0.0002   [ 0       0      1 ]*pi/180    ];

            obj.theta_Cov_pmpx = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpx), [5 5]);
            obj.theta_Cov_pmpy = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpy), [5 5]);
            obj.theta_Cov_pmpz = 0.00005*covarianceStomp(numel(obj.theta_mean_pmpz), [5 5]);

            obj.consolidateTheta();   
            
            
            obj.theta_mean =[];
            obj.theta_Cov  =[]; 
            obj.theta_mean_pert =[];
            
            obj.XYZrollOut =[];
            obj.refTrajPerturbed =[];


            obj.theta_mean_pert_Frame =[];
            obj.theta_mean_pert_pmpx =[];
            obj.theta_mean_pert_pmpy =[];
            obj.theta_mean_pert_pmpz =[];       
            
        end        

        
    end
    
end

