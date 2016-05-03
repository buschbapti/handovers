classdef Dmp < handle
% DMP implementations
%
% The type 'Hoffmann' follows the paper
% Hoffmann, H.; Pastor, P.; Park, D.-H. & Schaal, S. 
% Biologically-inspired dynamical systems for movement generation: 
%     automatic real-time goal adaptation and obstacle avoidance Robotics and Automation, 2009. 
%     ICRA'09. IEEE International Conference on, 2009, 2587-2592
%     
% The original follows the traditional DMP formulation
%
%
% Seems to be the best implmenetation for point-to-point case.
% If you need something with non-zero velocities, the work of Kober
% (hitting and batting) may be better.
%

    properties
        demoOriginal
        
        xi 
        xdi
        xf
        w  
        dt 

        y   
        yd  
        ydd 

        A
        
        timeFactorForSteadyState
        Gn
        Z
        
        forcingFunctionScaling
        nTraj
        Dgain
        Pgain
        nBasis
        
        type 
        
    end
    
    methods
        function obj = Dmp(demoOr, type, param)
            if nargin < 3
                param = [];
            end

            if ~isfield(param, 'xi');
                obj.xi = demoOr(1);
            else
                obj.xi = param.xi;
            end
            
            if ~isfield(param, 'forcingFunctionScaling')
                if strcmp(type, 'hoffmann')
                    obj.forcingFunctionScaling = 1; % Hoffmann method this should be on by default
                end
                if strcmp(type, 'original')
                    obj.forcingFunctionScaling = 0; % normal method is disabled
                end                
            else
                obj.forcingFunctionScaling = param.forcingFunctionScaling;
            end 
            
            if ~isfield(param, 'nTraj');
                obj.nTraj = 200; % assume 300 as a magic number
            else
                obj.nTraj = param.nTraj;
            end
            
            if ~isfield(param, 'Dgain')
                obj.Dgain = 100;
            else
                obj.Dgain = param.Dgain;
            end
            if ~isfield(param, 'Pgain')
                obj.Pgain = 1000;
            else
                obj.Pgain = param.Pgain;                
            end
            if ~isfield(param, 'nBasis')
                obj.nBasis = 20; % magic number
            else
                obj.nBasis = param.nBasis;                
            end            
            obj.demoOriginal = demoOr;
            
            obj.type = type;
            
            % Select which type of dmp
            if strcmp(type, 'hoffmann')
                 obj = obj.regression_Hoffmann;
            end
            if strcmp(type, 'original')
                 obj = obj.regression_original;
            end  
            
        end
                
        
        function obj = regression_Hoffmann(obj)            
            nTraj  = obj.nTraj;
            demoOr = obj.demoOriginal;
        
            % get the trajectory, resample to obj.nTraj if not yet.
            tnew    = linspace(0,1,nTraj);            
            if nTraj ~= numel(demoOr)
                tdemoOr = linspace(0,1,numel(demoOr));
                demo    = interp1(tdemoOr, demoOr, tnew);
            else
                demo = demoOr;
            end

            % get the derivatives
            dt = tnew(2)-tnew(1);
            demo_d  = diff(demo)/dt;
            demo_d  = [demo_d demo_d(end)];
            demo_dd = diff(demo_d)/dt;
            demo_dd = [demo_dd demo_dd(end)];     


            %% Regression
            if obj.forcingFunctionScaling
                A =  demo(end)-demo(1) ;
            else
                A = 1; % no scaling
            end
            target = demo(end).*ones(size(demo));
            % compute the transformation function values: canonical system
            Z = exp(-[1:numel(demo)]/(numel(demo)*0.2 )) ;

            Kp = obj.Pgain;
            Kd = obj.Dgain;
            
            fdemo  = demo_dd - Kp*(target - demo) + Kd*demo_d + Kp*A.*Z;
            fdemo  = fdemo/Kp;            
           
            obj.make_basis();
            Gn = obj.Gn;
                                              
            MPPI = (Gn'*Gn  + 1e-8*eye(numel(Gn(1,:)),numel(Gn(1,:)))) \ Gn'; 
            w = MPPI*(fdemo./Z)';
            
            obj.w  = w;
            obj.dt = dt;

            obj.y   = demo(1);
            obj.yd  = demo_d(1);
            obj.ydd = demo_dd(1);

            obj.A = A;
            obj.Z = Z;            
        end
        

        function ycut =  generalize(obj, param)
            switch obj.type
                case 'hoffmann'
                    ycut =  obj.generalize_hoffmann(param);
                case 'original'
                    ycut =  obj.generalize_original(param);                    
            end
        end
        
        function y =  generalize_hoffmann(obj, param)
 
            if nargin < 2
                param = [];
            end

            if isfield(param, 'xi')
                obj.xi = param.xi;
            end
            if isfield(param, 'xf')
                obj.xf = param.xf;
            else
                obj.xf = obj.demoOriginal(end);
            end            
            if ~isfield(param, 'xdi')
                obj.xdi = (obj.demoOriginal(2)-obj.demoOriginal(1))./obj.dt; 
            end
            if ~isfield(param, 'timeFactorForSteadyState');
                % whichever comes first will stop the loop
                obj.timeFactorForSteadyState = 2.5; % maximum time allowed
                minError = 0.05; % minimum error to stop the integration
            else
                minError = -999;
            end

            xi  = obj.xi;  % initial condition
            xdi = obj.xdi; % initial velocity condition
            xf  = obj.xf;  % final condition

            dt = obj.dt;
            
            % time for motor primitves        
            nTrajExtended = round(obj.nTraj*obj.timeFactorForSteadyState);

            % initialize motor primitive
            hmp.z = 1;            
            % - initial postion and velocity
            x1      = xi;  
            x2      = xdi; 
            target  = xf;

            y   = obj.y;
            yd  = obj.yd;
            ydd = obj.ydd;

            Kp = obj.Pgain;
            Kd = obj.Dgain; 
            
            if obj.forcingFunctionScaling
                A = xf-xi; 
            else
                A = 1 ; 
            end            
            alpha_z = 5;
            
            % Integrate
            tsum = 0;
            for i = 1:nTrajExtended

                tsum = tsum + dt;
                
                if i <= obj.nTraj
                    z_ = obj.Z(i);
                    f = obj.Gn(i,:)*obj.w*z_;                    
                else
                    f = 0;
                    z_ = 0;
                end

                hmp.x2_d  = Kp*(target-x1) -Kd*x2 -Kp*A*z_ + Kp*f;
                hmp.x1_dd = hmp.x2_d;
                hmp.x1_d  = x2;
                hmp.zd    = -alpha_z*hmp.z;
                
                x2 = x2 + dt*hmp.x2_d;
                x1 = x1 + dt*hmp.x1_d;
                
                hmp.z = hmp.z + dt*hmp.zd;

                % store values
                y(i)   = x1;
                yd(i)  = hmp.x1_d;
                ydd(i) = hmp.x1_dd;

                dbg=0;
                if 0
                    plot(i, y(i), sty('b', 'o', 2, [], 10) );        
                end
                if ( abs(target-y(i)) < minError ) && ( i > obj.nTraj )
                    if dbg
                        plot(i, target, sty('r', 'o', 2, [], 10) ); 
                    end
                    break
                end
            end
        end  
        
        function obj = regression_original(obj)
            
            nTraj  = obj.nTraj;
            demoOr = obj.demoOriginal;
        
            % get the trajectory, resample to a reasonable number 
            tnew    = linspace(0,1,nTraj);            
            if nTraj ~= numel(demoOr)
                tdemoOr = linspace(0,1,numel(demoOr));
                demo    = interp1(tdemoOr, demoOr, tnew);
            else
                demo = demoOr;
            end

            % get the derivatives
            dt = tnew(2)-tnew(1);
            demo_d  = diff(demo)/dt;
            demo_d  = [demo_d demo_d(end)];
            demo_dd = diff(demo_d)/dt;
            demo_dd = [demo_dd demo_dd(end)];     


            %% Regression
            if obj.forcingFunctionScaling
                A =  demo(end)-demo(1) ;
            else
                A = 1; % no scaling
            end

            target = demo(end).*ones(size(demo));

            % compute the transformation function values: canonical system
            Z = exp(-[1:numel(demo)]/(numel(demo)*0.2 )) ;
            
            Kp = obj.Pgain;
            Kd = obj.Dgain;
            
            fdemo  = demo_dd - Kp*(target - demo) + Kd*demo_d;
            fdemo  = fdemo/abs(A);
           
            obj.make_basis();
            Gn = obj.Gn;

            MPPI = (Gn'*Gn  + 1e-8*eye(numel(Gn(1,:)),numel(Gn(1,:)))) \ Gn'; 
            w = MPPI*(fdemo./Z)';
            
            obj.w   = w;
            obj.dt = dt;

            obj.y   = demo(1);
            obj.yd  = demo_d(1);
            obj.ydd = demo_dd(1);

            obj.A = A;
            obj.Z = Z;
            
        end
        
        function y =  generalize_original(obj, param)
 
            if nargin < 2
                param = [];
            end

            if isfield(param, 'xi')
                obj.xi = param.xi;
            end
            if isfield(param, 'xf')
                obj.xf = param.xf;
            else
                obj.xf = obj.demoOriginal(end);
            end            
            if ~isfield(param, 'xdi')
                obj.xdi = (obj.demoOriginal(2)-obj.demoOriginal(1))./obj.dt; 
            end
            if ~isfield(param, 'timeFactorForSteadyState');
                % whichever comes first will stop the loop
                obj.timeFactorForSteadyState = 2.5; % maximum time allowed
                minError = 0.01; % minimum error to stop the integration
            else
                minError = -999;
            end

            xi  = obj.xi;  % initial condition
            xdi = obj.xdi; % initial velocity condition
            xf  = obj.xf;  % final condition

            dt = obj.dt;

            % time for motor primitves        
            nTrajExtended = round(obj.nTraj*obj.timeFactorForSteadyState);            

            % initialize motor primitive
            % - canonical system
            hmp.z = 1;
            % - initial postion and velocity
            x1      = xi;  
            x2      = xdi; 
            target  = xf;
            target_d = 0;

            %tau = obj.tau;

            y   = obj.y;
            yd  = obj.yd;
            ydd = obj.ydd;

            alpha_g  = obj.Dgain;
            beta_g   = obj.Pgain/alpha_g;    

            if obj.forcingFunctionScaling
                A = obj.forcingFunctionScaling*abs(xf-xi); 
            else
                A = obj.A ; 
            end
            tsum = 0;

            dbg=0;
            if dbg
                figurew
            end

            
            alpha_z = 5;
            for i = 1:nTrajExtended

                % move target
                target = target + target_d*dt;

                tsum = tsum + dt;
               
                if i <= obj.nTraj
                    z_ = obj.Z(i);
                    f = obj.Gn(i,:)*obj.w*z_;                    
                else
                    f = 0;
                    z_ = 0;
                end
                    
                hmp.x2_d = (alpha_g*(beta_g*(target-x1)-x2) + A*f);
                hmp.x1_d = x2;

                hmp.zd    = -alpha_z*hmp.z;
                hmp.x1_dd = hmp.x2_d;

                x2 = x2 + dt*hmp.x2_d;
                x1 = x1 + dt*hmp.x1_d;
                hmp.z = hmp.z + dt*hmp.zd;

                % store values
                y(i)   = x1;
                yd(i)  = hmp.x1_d;
                ydd(i) = hmp.x1_dd;

                if dbg
                    plot(i, y(i), sty('b', 'o', 2, [], 10) );        
                end
                if ( abs(target-y(i)) < minError ) && ( i >  obj.nTraj )
                    if dbg
                        plot(i, target, sty('r', 'o', 2, [], 10) ); 
                    end
                    break
                end
            end
        end        
                
        function obj = make_basis(obj)
            
            basisCenter = linspace(0,1, obj.nBasis);

            z   = linspace(0, 1, obj.nTraj);

            %> (z - basisCenter)
            z_minus_center = bsxfun(@minus, z', basisCenter);

            sigma = 0.05*ones(1,obj.nBasis); % kernel width
            at = bsxfun(@times, z_minus_center, 1./sigma);

            % computing and normalizing basis (order 0)
            basis     = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) );
            basis_sum = sum(basis,2);
            basis_n   = bsxfun(@times, basis, 1 ./ basis_sum);
            
            obj.Gn    = basis_n;
            
        end      
    end % methods
    
end

