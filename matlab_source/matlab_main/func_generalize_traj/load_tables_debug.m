function lookupTraj = load_tables_debug(folderName)

    folder = ['./' folderName];
        
    if strcmp(folderName, 'lookupTraj1')
        load([folder '/table1TEST_20151124_105551.mat']);
        lookupTraj{1} = solTraj1;
        
    end

    if strcmp(folderName, 'lookupTraj2')
        
        
        load([folder '/table2TEST_20151124_105606.mat']);
        lookupTraj{1} = solTraj2;
        
    end


end