
function obj = set_initial_Darias_rest_posture(obj)
    % ========================================================================
    % This is Darias official rest posture from the pendant script in
    % relation to the end-effector tip.
    %[Trest, qRest] = set_initial_Darias_rest_posture(vrep, vrepm);

    xyz = [+0.6093    -0.4963    -0.1892];
    rpy = d2r([-117.87   +053.97   +159.09]);

    % 1. set cartesian target 
    obj.sendTargetCartesianCoordinates(xyz, rpy, obj.entt.handle(strcmp('Dummy_target', obj.entt.name)), 0);

    % 2. get rest posture    
    if 0 % use this go get a "ground truth" rest posture. This "ground truth" set is comprised of both 
         % Cartesian and joint configurations.

        % Because it is not guaranteed that the sent position can be achieved,
        % I read the actual one and use it as a rest posture.
        [~, ~, Trest, qRest] = read_generic_coordinates(vrep, vrepm.clientID, vrepm.idx.dummyTip);


        % check if hand achieved target. Otherwise I cannot use the achieved joint angles to impose a 
        % rest posture.
        % ===========================================
        currentPos     = Trest(1:3,4);
        currentOri     = tr2rpy(Trest(:,:));

        if sum(abs(currentPos-xyz')) > 0.01
            error('could not set XYZ of end-effector to desired rest posture');
        end
        if sum(abs(   r2d(currentOri) - r2d(rpy)  )) > 1
            error('could not set orientation of end-effector to desired rest posture');
        end

        save('restPosture0.mat', 'Trest', 'qRest');

    else % reload desired qRest which were checked to lead to a rest configuration.
        load('restPosture0.mat');
    end

    obj.qRestPosture = qRest; % it is better to impose this configuration by setting joint angles
                                     % directly. Setting rest posture via IK may get the arm stuck
                                     % in local minima.
    obj.TrestPosture = Trest;        
    obj.backToRestPosture(); 
end