function []= recover_java_crash()


    global optitrack;
    try
        optitrack.stop();
        optitrack.delete();
        optitrack = [];
    catch E2
        fprintf('Could not delete Optitrack.\n')
    end

    
    
    for k=1:10
        try
            dbquit
        catch E
            %       fprintf('dbquit\n')
        end
    end
    
    disp('Java crash finished');
    
end