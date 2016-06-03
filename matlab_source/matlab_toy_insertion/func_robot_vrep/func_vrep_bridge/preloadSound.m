function out = preloadSound

    out.name = {'ready'};
    [out.data{1}.soundY, out.data{1}.soundFs] = audioread('audio/ready2.wav'); 

    out.name{end+1} = 'hand_is_busy';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/screwdriverWithRobot.mp3');  
    
    out.name{end+1} = 'handing_plate';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/handingPlate.mp3');
    
    out.name{end+1} = 'handing_screw';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/handingScrew.mp3');
    
    out.name{end+1} = 'screwdriver_on_stand';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/screwdriverOnStand.mp3');
    
    out.name{end+1} = 'taking_screw_driver_from_human';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/takingScrewdriver.mp3');    
    
    out.name{end+1} = 'returning_the_screw_driver_to_the_human';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/handingScrewdriver.mp3');       

    out.name{end+1} = 'initial_home_position';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/movingToHomePosition.mp3');       
    
    out.name{end+1} = 'beep';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/beep_out.wav');           
    
    out.name{end+1} = 'error';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/error.wav');  
    
    out.name{end+1} = 'predicting_plate';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/predicting_plate.mp3');  
    
    out.name{end+1} = 'predicting_screw';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/predicting_screw.mp3');  
    
    out.name{end+1} = 'screw_predicted_but_stand_empty';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/screw_predicted_but_stand_empty.mp3');      
    
    % Returning from wrong prediction.
    out.name{end+1} = 'returning_from_wrong_prediction';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/returning_from_wrong_prediction.mp3');      
    
end


