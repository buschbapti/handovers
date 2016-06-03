function reps = userParamREPS(nUpdates, nRollOut)

    reps.nRollOut = nRollOut; % update the parameters after n_rollOouts  were sampled.
    
    
    reps.nUpdates = nUpdates; % use n_rollOouts to estimate the weights for the ML update.
    
    reps.noise_scale = 1;
    reps.no_noise    = 0;

    
    reps.KLBound     = 0.9;

    reps.n_reuse     = 0;
    
    reps.decreaseCovarianceAsPi2  = 1; % disable REPS updating of covariance and use the Pi2
                                            % approach of linearly decreasing the noise.
                                            
    reps.minNoiseMultiplierValue  = 0.0051; % the minimum noise scaling factor that is achieved by 
                                               % the final iteration. Note that this value is only
                                               % effective when decreaseCovarianceAsPi2 is active.
                                          
    reps.useSTOMPCovariance     = 1; % Stomp-type of covariance
    
    % Initial value to scale the covariance matrix.  Heavily problem dependent.
    reps.covarInitialScale      = 0.00005; % scales the A matrix for stomp-like covariance.
    reps.noiseCovarianceDiagonalOnly = 0; % if 0: use correlated noise, otherwise only diagonal.
    
    
    % Anchor weights
    % This disables the updating of the first and last weights. It is suitable for STOMP like
    % problems where the initial and final states are given.
    % If you need to explore the initial and final states you must set them to zero and set the
    % parameters "shakeInitial_nWeights" and "shakeFinal_nWeights" accordingly.
    % This can be seen as a hack, but it makes the solution converge much faster and towards better
    % solutions.
    %%reps.anchorInitial_nWeights = 0;
    %%reps.anchorFinal_nWeights   = 1;
    
    reps.clipStompCovariance = [5 0];
    
end



























