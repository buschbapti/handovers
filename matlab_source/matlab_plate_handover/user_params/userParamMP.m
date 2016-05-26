function mp = userParamMP(minCost, init, pickPosition, viap_xyz, viap_xyzdot)
% function mp = userParamMP(minCost, init, pickPosition, viap_xyz, via_xyzdot)
%
%   viap_xyz = [3x1]: specify a desired offset in relation to the coordinates of the position picked
%                     with ginput
%                     For entries NaN, it will use the original values. For example, 
%                     viap_xyz = [ NaN 3 NaN ] an offset in the Y direction will be applied, while X
%                     and Z will use the original value.


    mp.minCost = minCost; % the cost that will stop the optimization
    
    if pickPosition
        
        q    = NaN.*ones(numel(init.p(1,:)),3);
        qdot = NaN.*ones(numel(init.p(1,:)),3);
        
        figurewe('Pick a Position with your mouse');
        xlabel 'x'; ylabel 'z'; hold on; grid on;
        plot(init.p(1,:), init.p(3,:),  styw('b', 'o', [], '-', 10 ));
        screenxz = ginput(1);
        [v, i] = min( (init.p(1,:)- screenxz(1)).^2 + (init.p(3,:)- screenxz(2)).^2 );
        plot(init.p(1,i), init.p(3,i), sty('r', 'o', [], '-', 12 ));
        
        if ~isempty(viap_xyz)
            fprintf('original via point at specified index:\n   %g,  %g,  %g.\n', init.p(:,i));            
            for j=1:3       
                q(i,j) = init.p(j,i) + viap_xyz(j); 
            end            
            fprintf('modified via point at specified index:\n   %g,  %g,  %g.\n', q(i,:));
        end
        
        if ~isempty(viap_xyzdot)
            fprintf('original via point at specified index:\n   %g,  %g,  %g.\n', init.pdot(:,i));            
            for j=1:3
                qdot(i,j) = 0.*init.pdot(j,i) + viap_xyzdot(j);
            end            
            fprintf('modified via point at specified index:\n   %g,  %g,  %g.\n', qdot(i,:));
        end
        
        mp.viaPoint.q    = q;
        mp.viaPoint.qdot = qdot;

        save('mp_presaved.mat', 'mp');
    
    else % reload previous points
        
        load('mp_presaved.mat');
        
    end  
    
end

% 
%     q    = NaN.*ones(numel(init.p(1,:)),3);
%         qdot = NaN.*ones(numel(init.p(1,:)),3);
%         
%         figurewe('Pick a Position with your mouse');
%         subplot(2,1,1), xlabel 'x'; ylabel 'z'; hold on; grid on;
%         plot(init.p(1,:), init.p(3,:),  styw('b', 'o', [], '-', 10 ));
%         screenxz = ginput(1);
%         [v, i] = min( (init.p(1,:)- screenxz(1)).^2 + (init.p(3,:)- screenxz(2)).^2 );
%         
%         
%         subplot(2,1,2), xlabel 'x'; ylabel 'y'; hold on; grid on;
%         plot(init.p(1,:), init.p(2,:),   styw('b', 'o', [], '-', 10 ));
%         plot([init.p(1,i)  init.p(1,i)], [min(init.p(2,:))-3 max(init.p(2,:))+3],  styw('r'));
%         screenxy = ginput(1);        
%          
% 
%         
%         plot3(init.p(1,i), init.p(2,i), init.p(3,i), sty('r', 'o', [], '-', 12 ));
%              
