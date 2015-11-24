function out = preloadSound_Baxter

try
    out.name = {'ready'};
    [out.data{1}.soundY, out.data{1}.soundFs] = audioread('audio/ready2.wav');
    
    out.name{end+1} = 'step_one';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/step_one.mp3');    
    
    out.name{end+1} = 'step_two';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/step_two.mp3');
    
    out.name{end+1} = 'step_three';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/step_three.mp3');
    
    out.name{end+1} = 'trajectory_generated';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/trajectory_generated.mp3');        
    
    out.name{end+1} = 'poses_acquired';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/poses_acquired.mp3');      
    
    out.name{end+1} = 'generating_trajectory';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/generating_trajectory.mp3');
    
    out.name{end+1} = 'do_you_want_to_see_final_sol';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/do_you_want_to_see_final_sol.mp3');    
    
    out.name{end+1} = 'do_you_want_to_see_final_sol';
    [out.data{end+1}.soundY, out.data{end+1}.soundFs] = audioread('audio/animating_final_solution.mp3');        
    
catch
   out.name = 'no audio reader available'; 
end
end


