function demo1 = prepareFirstHalfData(demo)

    demo1.newVP.idx = demo.newVP.idx(1);
    demo1.newVP.T   = demo.newVP.T(:,:,1);
    demo1.newVP.p   = demo.newVP.p(:,1);
    
    
    demo1.robot.T = demo.robot.T(:,:,1:end-1);
    demo1.robot.p = squeeze(demo1.robot.T(1:3,4,:));
    
end