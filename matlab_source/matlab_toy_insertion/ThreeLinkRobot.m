classdef ThreeLinkRobot < handle
% Specific functions for the letterA
    
    properties
        para
        nTraj = 10;
        restPostureT
        paramIK
        costIK
        costIK_clean
    end
    methods

        function obj = ThreeLinkRobot()
            obj.para = RobotRRR(); 
            [T_tmp ] = obj.FK_and_homog_transf(obj.para.restAngles');
            obj.restPostureT =  T_tmp{end};
            
            
            % ====================================================
            % This set will change the low-level solver. Avoid tuning these
            % numbers!
            % set default parameters for solution of inverse kinematics
            obj.paramIK.ilimit    = 5;
            obj.paramIK.tol       = 1e-6;
            obj.paramIK.alpha     = 0.5;   % initial value to start learning

            obj.paramIK.varstep   = true;    % variable step
            obj.paramIK.alpha_max = 0.5;   % gjm: saturate max rate to improve
            obj.paramIK.alpha_min = 0.001; % gjm: saturate min rate to improve
            % ==========================================================

            obj.paramIK.plot        = false;  % this will plot some values useful to debug

            obj.paramIK.pinv          = 1; % pinv is usually much better than transpose
            obj.paramIK.verbose = 0;
            obj.paramIK.pInvNoise = 0.01;
            obj.paramIK.pinv_restPost = 0; % pinv is usually much better than transpose            
%            obj.paramIK.nResampleError = obj.paramIK.createInitTraj.nSamples + obj.paramIK.connectTraj.maxSteps;
%            obj.paramIK.minCost = ikCost; % criterion to stop ikne solution in the outer loop
                
        end

        
        function [ikerrorTraj, h] = IKcost(obj, j, flagClean, h, xyz, refOrientation)
                                 %  j, (j-nRollOut), h, DMP.XYZrollOut(:,:,:,j), DMP.refOrientation
          
            % quickly resample the trajectory to make IK faster
            dt = round(numel(xyz(1,:))/obj.nTraj);
            id = 1:dt:numel(xyz(1,:));
            if id(end)~=numel(xyz(1,:))
                id(end+1) = numel(xyz(1,:));
            end
            k=1;
            for i=id
                T2(:,:,k) =  [   [refOrientation(1:3,1:3, i);[0 0 0]]    [xyz(:,i);1]];
                k=k+1;
            end
            
            % add connecting trajectory
            T_connect  = makeConnectingTrajectory(obj, T2(:,:,1), 5);
            T2 = cat(3, T_connect, T2);
            
            %homogTransfPlot(T2, struct('hfig', h.fig))
            
            
            % =====================================
            % ik on the currentT    
            qi = obj.para.restAngles';    
            ik_hist =  obj.my_ikine(T2, obj.para.restAngles');
            
            if 0    
                figure(h.fig);
                h.del = []; 
                nTraj = numel(ik_hist.T(1,1,:));
                for k=1:nTraj
                    [T_, p_ ] = obj.FK_and_homog_transf( ik_hist.q(k,:) );
                    h.del = [h.del  plot(p_(:,  1), p_(:,  2), styw('k', 'o', 4,' -',10 )  )];
                    title(['Step ' num2str(k) ' of '  num2str(nTraj)]);       
                    drawnow;
                    pause(0.4) %keyboard
                    if  1% k ~= numel(traj.q_ik(:,1))
                        delete(h.del); h.del=[];
                    end
                end                
            end
            
            
            % =====================================
            %ik_hist.neResampled = interp1(linspace(0,1, numel(ik_hist.ne)),ik_hist.ne,   linspace(0,1,param.ikine.nResampleError));
            obj.costIK(j) = sum(ik_hist.ne.^2);
           
            if flagClean == 0
                obj.costIK_clean      = [obj.costIK_clean     obj.costIK(j)]; 
                figure(h.fig);
                xyzClean = squeeze(ik_hist.T(1:3,4,:));
                h.plotCartIK = [h.plotCartIK plot3(xyzClean(1,:), xyzClean(2,:), xyzClean(3,:), sty('b', [], 2))];
                
            end
            ikerrorTraj = ik_hist.ne;            
        end
        
        
        function [hist, hist_total] = my_ikine(obj, tr, q)
        %[hist, hist_total] = my_ikine(robot, tr, q, mask, opt)

            opt = obj.paramIK;
            robot = obj.para;
            mask = obj.para.mask;
            
            robot.nDoF = robot.nDoF;

            % 2dn output argument. History at each time step.
            hist.ne = []; hist.T =[];

            % 3rd output argument. Total history
            hist_total.q = []; hist_total.dq = []; hist_total.e = []; hist_total.ne = []; hist_total.alpha = [];

            % make this a logical array so we can index with it
            mask = logical(mask);

            npoints = size(tr,3);    % number of points
            qt = zeros(npoints, robot.nDoF);  % preallocate space for results
            tcount = 0;              % total iteration count

            J0  = obj.analyticalRobotJacobian( robot, q  );
            % cut the jacobian according to the mask
            J0 = J0(mask, :); % but in PC code!!

            if cond(J0) > 100
                warning('IK: Your initial guess results in near-singular configuration, this may slow convergence');
            end

            history = [];     

            [NJ1, NJ2] = size(J0);
            I = eye(NJ1, NJ2);

            for t = 1:npoints

                if opt.verbose
                    fprintf('Step %g of %g\n', t, npoints);
                end
                T = tr(:,:,t);

                nm = Inf;
                % initialize state for the ikine loop
                eprev = Inf;
                save.e = [Inf Inf Inf Inf Inf Inf];
                save.q = [];
                count = 0;

                while true

                    % update the count and test against iteration limit
                    count = count + 1;

                    if count > opt.ilimit
                        if opt.verbose
                            fprintf('ikine: %d iter. exceeded. Final err %f\n', opt.ilimit, nm); 
                        end
                        if 0
                            q = NaN*ones(1,robot.nDoF);
                        end

                        break
                    end

                    % saturate angles according to the robot joint limit
                    checkLim = bsxfun(@minus,q',robot.jointLimit);
                    iL       = checkLim(:,1)<0;
                    q(iL)    = robot.jointLimit(iL,1);
                    iR = checkLim(:,2)>0;
                    q(iR) = robot.jointLimit(iR,2);

                    % compute the error
                    T_ = obj.FK_and_homog_transf( q );
                    e = tr2delta( T_{end}, T);


                    % optionally adjust the step size
                    if opt.varstep
                        % test against last best error, only consider the DOFs of
                        % interest
                        if  norm(e(mask)) < norm(save.e(mask))
                            % error reduced,
                            % let's save current state of solution and rack up the step size
                            save.q = q;
                            save.e = e;
                            if opt.alpha <= opt.alpha_max
                                opt.alpha = opt.alpha * (2.0^(1.0/8));
                            end                    
                            if opt.verbose
                                fprintf('raise alpha to %f\n', opt.alpha);
                            end
                        else
                            % rats!  error got worse,
                            % restore to last good solution and reduce step size
                            q = save.q;
                            e = save.e;
                            if opt.alpha >= opt.alpha_min
                                opt.alpha = opt.alpha*0.5;
                            end
                            if opt.verbose
                                fprintf('drop alpha to %f\n', opt.alpha);
                                fprintf('Restoring last solution and reducing alpha.\n');
                            end

                        end
                    end

                    if opt.alpha == 0
                       keyboard 
                    end

                    % compute the Jacobian
                    J  = obj.analyticalRobotJacobian( robot, q  );

                    % compute change in joint angles to reduce the error, 
                    % based on the square sub-Jacobian            
                    if opt.pinv

                        J_pinv = pinv(J(mask,:) + opt.pInvNoise.*I);

                        dq = opt.alpha*J_pinv*e(mask);

                        if opt.pinv_restPost
                            JinvJ = J_pinv*J(mask,:);
                            dq = dq + pinv( eye(size(JinvJ)) - JinvJ  )*robot.restAngles;
                        end

                    else
                        dq = J(mask,:)'*e(mask);
                        dq = opt.alpha * dq;
                    end

                    % diagnostic stuff
                    if opt.verbose
                        fprintf('%d:%d: |e| = %f\n', t, count, nm);
                        fprintf('       e  = '); disp(e');
                        fprintf('       dq = '); disp(dq');
                    end
                    if nargout > 2
                        hist_total.q  =  [hist_total.q    q'];
                        hist_total.dq =  [hist_total.dq  dq ];
                        hist_total.e  =  [hist_total.e  e];
                        hist_total.ne =  [hist_total.ne  nm];
                        hist_total.alpha = [hist_total.alpha opt.alpha];
                    end

                    % update the estimated solution
                    q = q + dq';
                    nm = norm(e(mask));

                    if norm(e) > 1.5*norm(eprev)
                        warning('RTB:ikine:diverged', 'solution diverging, try reducing alpha');
                    end
                    eprev = e;

                    if nm <= opt.tol
                        break
                    end

                end % while loop =========== 


                hist.ne        = [hist.ne  nm];
                hist.T(:,:,t)  = T_{end};

                qt(t,:) = q';
                tcount = tcount + count;
                if opt.verbose
                    fprintf('%d iterations\n', count);
                    fprintf('=================\n\n\n', count);            
                end

            end % for each point in the trajectory
            hist.q = qt;


            if opt.verbose && npoints > 1
                fprintf('TOTAL %d iterations\n', tcount);
            end



            % plot evolution of variables
            if opt.plot
                figurew('history q');
                plot([history.q]');
                xlabel('iteration');
                ylabel('q');
                grid

                figurew('dq');
                plot([history.dq]');
                xlabel('iteration');
                ylabel('dq');
                grid

                figurew('error');
                plot([history.e]');
                xlabel('iteration');
                ylabel('e');
                grid

                figurew('abs error semilog');
                semilogy([history.ne]);
                xlabel('iteration');
                ylabel('|e|');
                grid

                figurew('alpha');
                plot([history.alpha]);
                xlabel('iteration');
                ylabel('\alpha');
                grid

            end
        end
        
        
        function Tout = makeConnectingTrajectory(obj, T2, maxSteps)

            %maxSteps = param.maxSteps;

            % This function should be improved to provide an interpolation between the initial posture to
            % the 
            %T1_ = FK_and_homog_transf( robot.L, robot.alpha, robot.d, robot.restAngles');
            T1 = obj.restPostureT;

            % Heuristic to find number of steps
            dist = obj.distance( T1(1:3,end)', T2(1:3,end)'  );

            nSteps = floor( (maxSteps/1)*dist ); % 50/1: 50 points for every 1 meter of trajectory

            if nSteps > maxSteps % saturate here
                nSteps = maxSteps;
            end
            if nSteps <= 2
                nSteps = 4;
            end
            Tout = interpolate_two_quatertions(T1, T2, nSteps);
        end
        
        
        function plotRestPosture(obj)
            [~, p_tmp ] = obj.FK_and_homog_transf(obj.para.restAngles');
            traj_tmp(:,:,1) = p_tmp;  % this holds 
            plot(traj_tmp(:,1,1) ,   traj_tmp(:,2,1),    sty('k', 'o', 2, '-', 7) );
            plot(traj_tmp(end,1,1) , traj_tmp(end,2,1), styw('k', 'o', 2, '-', 10) )
            clear traj_tmp;            
        end
  
        function  [Tout, xyz_robot] = FK_and_homog_transf(obj, q)
        % Homogeneous Transformation Matrix from all links starting at the base.
        % 
        % INPUT
        %     L     [Mx1] is a vector of link M lengths
        %     alpha [Mx1] is a vector of M relative rotations of the z axis around X
        %              (see Khatib's notes or any book on DH)
        %     d     [Mx1] is a vector of M relative distances between consecutive z
        %              axes (see Khatib's notes or any book on DH)
        %     q     [Mx1] is a vector of positions
        %
        % OUTPUT
        %
        %   Tout{M} is a vector of cells, where each cell representes the
        %   homogeneous transformation from base 0 to link m, where m = {1...M}
        %
        %   xyz_robot{M} is a vector with the position of a point at the end of
        %                each link, starting from the base. It is useful to plot
        %                the whole arm by just doing 
        %                plot3(xyz(:,1), xyz(:,2), , xyz(:,2))
        %
        %
        %  For example, for a  two link arm RR with length L1 and L2. Following Khatib's Stanford lecture
        %  notes0.
        %
        %   i     alpha_{i-1}    L_{i-1}   d_i   q_i
        %   --------------------------------------------------
        %   1          0            0       0     q1
        %   2          0            L1      0     q2
        %   3          0            L2      0     0
        %
        %
        % 20.02.2015  Guilherme  v00: function created
        %
        %

            alpha = obj.para.alpha;
            L = obj.para.L;
            d = obj.para.d;
        
            % I had to augment the vectors to make it compliant with Khatib's
            % notation. Maybe this is the reason why there is another DH convetion.
            q     = [q 0]; 
            d     = [d 0]; 
            alpha = [0 alpha];
            L     = [0 L];

            xyz_robot = [];
            for k=1:numel(q);
                T{k} = obj.principal_formula( q(k), d(k), alpha(k), L(k) );
                if k==1
                    Tout{k} = T{k};
                else
                    Tout{k} = Tout{k-1}*T{k};
                end
                tmp = Tout{k}*[0 0 0 1]';
                xyz_robot = [xyz_robot; tmp(1:3)'];
            end

        end

        function [Jacobian_analytical] = analyticalRobotJacobian(obj, r, q  )

            [T_unp ] = obj.FK_and_homog_transf(  q );
            R = T_unp{end}(1:3,1:3);

            % derivative in relation to 
            dq = 1e-6;
            Jacobian_analytical = zeros(6, r.nDoF);
            zeros_pert = zeros(1,r.nDoF);
            for j = 1:r.nDoF

                % perturb the dof(j)        
                q_pert    = zeros_pert;
                q_pert(j) = dq;
                q_pert = q + q_pert;

                [T] = obj.FK_and_homog_transf( q_pert);

                % Difference in translation
                dTdq  = (T{end}-T_unp{end}) / dq ;
                jxyz  = dTdq(1:3,end);

                % Difference in rotation (this comes from Peter Corke code)
                % Check function tr2delta.m which could be used instead.
                dRdq = dTdq(1:3,1:3);

                J = [ jxyz ; vex( dRdq*R' ) ];

                Jacobian_analytical(:,j) = J;

            end        
        end
        
    end 

    methods(Static)


        function [T] =  principal_formula( q, d, alpha, L)
        % Check Khatibs's lecture notes. Eq 2.6
        % Each transformation is of the form
        %     k=1;
        %     T_0to1 = [  cos(q)                    -sin( q )                        0              L(k-1)
        %                sin(q)*cos(alpha(k-1))   cos(q)*cos(alpha(k-1))  -sin(alpha(k-1))  -sin(alpha(k-1))*d
        %                sin(q)*sin(alpha(k-1))   cos(q)*sin(alpha(k-1))   cos(alpha(k-1))   cos(alpha(k-1))*d
        %                 0                             0                                   0            1              ];
        %
        %
            T  =     [  cos(q)                    -sin( q )        0              L
                       sin(q)*cos(alpha)   cos(q)*cos(alpha)  -sin(alpha)  -sin(alpha)*d
                       sin(q)*sin(alpha)   cos(q)*sin(alpha)   cos(alpha)   cos(alpha)*d
                        0                             0              0            1              ];

        end


        function [dist] = distance( point1, point2)
        %
        % calculates the distance between 2 points.
        % Each point has dimension N.
        % point1 =  [x1 x2 x3 ... xn];
        %
        %  Example: point1 is 2 dimensional with coordinates XY
        %           point1 = [3 4];
        % 
        % 4 july 2010: gjm: file created

        N1_size = size(point1, 2);
        N2_size = size(point2, 2);


            if N1_size ~= N2_size
                fprintf('Error: Given points have different dimension.\n ')
            else
                sqrt_of_all_differences = (point2 - point1).^2;
                dist = sqrt(  sum(sqrt_of_all_differences)  );
            end


        end




        function opt = get_standard_opt_values(nMax, pinv_flag)

            %  set default parameters for solution
            opt.ilimit = 1000;

            if ~isempty(nMax)
                opt.ilimit = nMax;
            end
            opt.tol       = 1e-6;
            opt.alpha     = 0.5;
            opt.alpha_max = 0.5;   % gjm: saturate max rate to improve
            opt.alpha_min = 0.01;  % gjm: saturate min rate to improve
            opt.plot = true;
            opt.varstep = true;

            if pinv_flag
                opt.pinv = true;
            else
                opt.pinv = false;
            end
            opt.pinv_restPost = 0;
            opt.verbose = 1;
            opt.pInvNoise = 0.0001;

        end
        

        
        
    end
    
        
        
    
   
end