classdef Probabilistic_DMP_shelf < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        
        y_orig
        dmp
        nDemo
        dmpReference
        w_mean
        w_Cov
        h   
            
    end
    
    methods
        
        function [obj] = Probabilistic_DMP_shelf(y, plotFig)

            param.nTraj = 300; 
            param.forcingFunctionScaling = 0;

            obj.nDemo  = numel(y(1,:));
            obj.y_orig = y;
            
            % create dmps
            for k=1:obj.nDemo                
                dmp{k} = dmp_regression( y(:,k), param );
            end
            obj.dmpReference = dmp_regression( 0*y(:,k), param );
            
            if plotFig
                obj.h = figurew('dmpTraj');
                plot( linspace(0,1,numel(y(:,1))), y   );
            end          
            
            % concatenate weights with goals
            wfull = [];
            for k=1:obj.nDemo
                wfull = [wfull ;  [dmp{k}.w'   obj.y_orig(end,k)  ]];
            end
            
            obj.w_mean = mean(wfull);
            obj.w_Cov  = cov(wfull);
            
        
        end
        
        function [ygen] = condition(obj, yObsInit, yObsFinl)
            
            n = numel(obj.w_mean);
            q = n-1;
            Cov11 = obj.w_Cov(1:q,1:q);
            Cov21 = obj.w_Cov(1:q,n);
            Cov12 = obj.w_Cov(n,1:q);
            Cov22 = obj.w_Cov(n,n);
            
            w_meanNew = obj.w_mean(1:q) + Cov22\Cov12*(  yObsFinl - obj.w_mean(n)  );
            
            generalized_dmp   = obj.dmpReference;
            generalized_dmp.w = w_meanNew';
            
            param.xi = yObsInit;
            param.xf = yObsFinl;
            ygen = dmp_generalize(generalized_dmp, param);

        end



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

