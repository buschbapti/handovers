function diagnose(e, qmin, qmax)

    fprintf('\n');
    error = 0;
    if e > 1e-4
        fprintf('Norm: %g. Desired posture may not be achievable.\n', e);
        error = error+1;
    end
    if sum(qmin) > 0
        fprintf('Hitting minimum joint limit during motion\n');
        error = error+1;
    end
    if sum(qmax) > 0
        fprintf('Hitting maximum joint limit during motion\n');
        error = error+1;
    end
    
    if error == 0
        fprintf('Generalization okay\n');
    end
    
end