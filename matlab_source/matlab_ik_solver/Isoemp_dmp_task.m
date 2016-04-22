classdef Isoemp_dmp_task < Isoemp_dmp
% Specific functions for the letterA
    
    properties

        cost_viaPoint
        cost_similarity
        cost_obstacle
        cost_total
        cost_clean
        cost_startEnd
        cost_odometry

    end
    methods

        function obj = Isoemp_dmp_task(viaPoint, obstacle, nTraj)
            obj@Isoemp_dmp(viaPoint, obstacle, nTraj);
        end
        
        function cost(obj, j, flagClean, w)
            % w: weights
            %   w(1): similarity
            %   w(2): obstacle
            %   w(3): via_point_InitialFinalStates
            %   w(4): via_point_IntermediatePoints

            xyz      = squeeze( obj.TrollOut(1:3,4,:,j) );
            letterAr = squeeze( obj.refTrajPerturbed(1:3,4,:,j) );
            
            % compare similarity here {R}
            Csimilarity = w(1)*obj.similarity(letterAr, xyz);

            % compute obstacle avoidance
            if ~isempty(obj.obst)
                Cobstacle = w(2)*obj.obstacle_avoid(xyz, obj.obst);
            else
                Cobstacle =0;
            end
            
            % via point start and end
            CstartEnd = w(3)*obj.via_point_start_end(obj, xyz);
            
            % via point intermediate values
            if w(4) ~= 0
                CviaPoint = w(4)*obj.via_point(obj, xyz);
            else
                CviaPoint = 0;
            end
            
            % odomoetry
            odometry = cumsum(abs(diff(xyz(1,:)))) + cumsum(abs(diff(xyz(2,:)))) + cumsum(abs(diff(xyz(3,:))));            
            Codometry = w(5)*sum(odometry);
            
            
            obj.cost_total(j) =  CstartEnd + Csimilarity + Cobstacle + CviaPoint + Codometry;


            
            
           
            if flagClean == 0
                obj.cost_clean      = [obj.cost_clean obj.cost_total(j)];
                obj.cost_startEnd   = [obj.cost_startEnd   CstartEnd];
                obj.cost_viaPoint   = [obj.cost_viaPoint   CviaPoint];
                obj.cost_similarity = [obj.cost_similarity Csimilarity];
                obj.cost_obstacle   = [obj.cost_obstacle   Cobstacle];
                obj.cost_odometry   = [obj.cost_odometry   Codometry];
            end
        end 
        
        
        function restart(obj)

            obj.cost_viaPoint=[];
            obj.cost_startEnd=[];
            obj.cost_similarity=[];
            obj.cost_obstacle=[];
            obj.cost_total=[];
            obj.cost_clean=[];
            obj.cost_odometry=[];
            
            obj.restart_main;
        end     
        
        
        
    end 

    methods(Static)

        function Csimilarity = similarity(trajA, trajB)

            distTraj =  sqrt( sum( (trajA-trajB).^2 ) ) ;
            
            Csimilarity = sum(distTraj.^2);

        end

        function Cobstacle = obstacle_avoid(x, obst )

           distanceToObstacle = bsxfun(@minus, x, obst.xyz)';
           distanceToObstacle = sqrt(sum(distanceToObstacle.^2,2));
           distanceToObstacle = max(  obst.rad-distanceToObstacle, 0   );

           Cobstacle = sum(distanceToObstacle.^2);
           %fprintf('Obs %g\n', Cobstacle)
           %disp(Cobstacle)
        end
        
        function CstartEnd = via_point_start_end(obj, xyz)            
            VPerror(1) = sqrt(sum( (obj.vp.init(1:3,4)' - xyz(1:3,1)').^2     )  );
            VPerror(2) = sqrt(sum( (obj.vp.finl(1:3,4)'-  xyz(1:3,end)').^2   )  );
            CstartEnd  = sum( VPerror.^2 );
        end
        
        function CviaPoint = via_point(obj, xyz)
            
            k=1;
            for n = obj.vp.mid.idx
                VPerror(k) = sqrt(   sum( (obj.vp.mid.xyz(k,:)' - xyz(1:3,n)).^2 ) );
                k=k+1;
            end
            CviaPoint  = sum( VPerror.^2 );
        end        
        
    end
    
        
        
    
   
end