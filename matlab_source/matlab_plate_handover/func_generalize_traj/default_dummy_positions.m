function [] = default_dummy_positions(robot, d_viaPoint, d_handover, reload)

    if reload
        load('default_dummy_positions.mat');
        robot.sendTargetCartesianCoordinates(tmpRobotTarget.T(1:3,4), tr2rpy(tmpRobotTarget.T), robot.getHandle('Dummy_target'), 1);                
        d_viaPoint.sendTargetCartesianCoordinates(tmpvp.T(1:3,4),   tr2rpy(tmpvp.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);
        d_handover.sendTargetCartesianCoordinates(tmpreba.T(1:3,4), tr2rpy(tmpreba.T), d_handover.getHandle('handoverPosition'), 1);
    else
        % read data from VREP and save
        [~,~, tmpRobotTarget.T]   = robot.readGenericCoordinates(robot.getIndex('Dummy_target'));        
        [~,~, tmpvp.T]   = d_viaPoint.readGenericCoordinates(d_viaPoint.getIndex('Dummy_viaPoint_table'));        
        [~,~, tmpreba.T] = d_handover.readGenericCoordinates(d_handover.getIndex('handoverPosition'));
        save('default_dummy_positions.mat', 'tmpvp', 'tmpreba', 'tmpRobotTarget');
    end

end


%robot.entt


