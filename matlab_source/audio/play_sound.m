function [] = play_sound(soundFiles, userInputString)


    k = find(strcmp(soundFiles.name, userInputString)); 
    if ~isempty(k)
        sound(soundFiles.data{k}.soundY, 1*soundFiles.data{k}.soundFs);
    else
        disp(userInputString);
    end

end