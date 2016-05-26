function [ik_hist, param] =  ikine_vrep( robot, T, pause_, param )
%     ne: [1x33 double]
%      T: [4x4x33 double]
%      q: [33x3 double]
%
%   pause_: if empty: stop and wait for keyboard input
%           if 0: do not pause at all
%           if not zero, use the value as pause duration (in seconds)
%
%   keep_last_posture: an optional flag that if given and set to 1 will maintain the last posture of
%                      the arm. It is only useful to make the animation more pretty. But should not
%                      be given when running the code for real.
%


    % storage variable
    st.target.xyz = [];
    st.target.rpy = [];
    
    st.actual.xyz = [];
    st.actual.rpy = [];
    st.actual.q   = [];
    
    st.initTarget.xyz = [];
    st.initTarget.rpy = [];
    
    mask = param.ikine.mask;

    nT = size(T,3);

    
    % independent of the trajectory, send it to the rest posture before starting any loop
    % The reasoning is that the location of the trajectory will favor being close to this posture
    robot.backToRestPosture( ); 

    handleDummyTarget  = robot.getHandle('Dummy_target');
    indexDummyTip      = robot.getIndex('Dummy_tip');
    
    for t = 1:nT
        % send target position
        xyz_ = T(1:3,4,t);  rpy_ = tr2rpy(T(:,:,t));        
        robot.sendTargetCartesianCoordinates(xyz_, rpy_, handleDummyTarget, 1);
        st.target.xyz = [ st.target.xyz;  xyz_' ];
        st.target.rpy = [ st.target.rpy;  rpy_' ];

        % get current tip position
        [xyz_, rpy_, Tcurr, qrobot_] = robot.readGenericCoordinates(indexDummyTip);
        st.actual.xyz = [st.actual.xyz; xyz_];
        st.actual.rpy = [st.actual.rpy; rpy_]; 
        st.actual.q   = [st.actual.q;   qrobot_];        

        e = tr2delta( Tcurr, T(:,:,t) );
        
        ik_hist.ne(:,t)  = norm(e (mask) );
        ik_hist.T(:,:,t) = Tcurr;
        
        if ~isempty(pause_)
            if pause_~=0
                pause(pause_);
            end
        else
            pause;
        end
       
    end

    ik_hist.q = st.actual.q;    
    
    startIndex = 1;
    ik_hist.ne = ik_hist.ne(startIndex:end);
    ik_hist.T  = ik_hist.T(:,:,startIndex:end);
    ik_hist.q  = ik_hist.q(startIndex:end,:);    
    
    if ~isempty(param.hfig.single_IK_iteration)
        
        figure(param.hfig.single_IK_iteration);
        
        try delete(param.hfig.single_IK_iterationTraj); end
        try delete(param.hfig.single_IK_iterationShadow); end
            
        targetP  = getPositionFromT(T);
        currentP = getPositionFromT(ik_hist.T);
        
        [htraj1, hshadow1] = plot_traj_with_shadow( targetP,  robot.vrepm.opt.plotShadowzheight, styw('b', 'o', 2, '-', 5), [] ) ;
        [htraj2, hshadow2] = plot_traj_with_shadow( currentP, robot.vrepm.opt.plotShadowzheight, sty('r', 'o', 2, '-', 5), [] ) ;
        legend({'Init', 'Target', 'IKSol'});
        
        
        param.hfig.single_IK_iterationTraj   = [htraj1 htraj2];
        param.hfig.single_IK_iterationShadow = [hshadow1 hshadow2];
  
        drawnow;
    end

end










