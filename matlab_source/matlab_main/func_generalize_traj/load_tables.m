function lookupTraj = load_tables(folderName)

    folder = ['./' folderName];
        
    if strcmp(folderName, 'lookupTraj1')
        %load([folder '/table1_20151111_113045.mat']);
        load([folder '/table1_20151130_121146.mat']);        
        lookupTraj{1} = solTraj1;
        
        %load([folder '/del1_20151112_104318.mat']);
        %lookupTraj{end+1} = solTraj1;       
    end

    if strcmp(folderName, 'lookupTraj2')
        
        %load([folder '/table2_20151111_113045.mat']);
        load([folder '/table2_20151130_115714.mat']);
        lookupTraj{1} = solTraj2;
        
        %load([folder '/del2_20151112_104318.mat']);
        %lookupTraj{end+1} = solTraj2;
        
    end


end