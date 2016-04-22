classdef Isoemp_pmp_letterA < Isoemp_pmp
% Specific functions for the letterA
    
    properties

        cost_viaPoint
        cost_similarity
        cost_obstacle
        cost_total
        cost_clean

    end
    methods

        function obj = Isoemp_pmp_letterA(xyzTraj, viaPoint, obstacle, opt)
            obj@Isoemp_pmp(xyzTraj, viaPoint, obstacle, opt);
        end
        
        function cost(obj, j, flagClean, w)
            % w: weights
            %   w(1): similarity
            %   w(2): obstacle
            %   w(3): via_point

            xyz      = obj.XYZrollOut(:,:,:,j);
            letterAr = obj.refTrajPerturbed(:,:,:,j);
            
            % compare similarity here {R}
            Csimilarity = w(1)*obj.similarity(letterAr, xyz);

            % compute obstacle avoidance
            Cobstacle = w(2)*obj.obstacle_avoid(xyz, obj.obst);

            
            % via point
            CviaPoint = w(3)*obj.via_point(obj, xyz);
            

            obj.cost_total(j) =  CviaPoint + Csimilarity + Cobstacle;

           
            if flagClean == 0
                obj.cost_clean      = [obj.cost_clean obj.cost_total(j)];
                obj.cost_viaPoint   = [obj.cost_viaPoint   CviaPoint];
                obj.cost_similarity = [obj.cost_similarity Csimilarity];
                obj.cost_obstacle   = [obj.cost_obstacle   Cobstacle];                
            end
        end
        
        function restart(obj)

            obj.cost_viaPoint=[];
            obj.cost_similarity=[];
            obj.cost_obstacle=[];
            obj.cost_total=[];
            obj.cost_clean=[];
            
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
        
        function CviaPoint = via_point(obj, xyz)
            
            VPerror(1) = sqrt(sum( (obj.vp.init' - xyz(1:3,1)').^2     )  );
            VPerror(2) = sqrt(sum( (obj.vp.finl'- xyz(1:3,end)').^2   )  );
            
            % implementation of intermediate points not yet done
            
            CviaPoint  = sum( VPerror.^2 );
            
            % 
            
        end
        
        
    end
    
        
        
    
   
end