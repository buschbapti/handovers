function out = resample_for_fast_IK(resampleIdx, run)

    out = run;
    out.p = run.p(:,resampleIdx); 
    out.T = run.T(:,:,resampleIdx);
    
end