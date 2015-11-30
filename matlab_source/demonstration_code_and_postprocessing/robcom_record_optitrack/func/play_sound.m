function [] = play_sound(soundFiles, userInputString)

    k = find(strcmp(soundFiles.name, userInputString));    
    sound(soundFiles.data{k}.soundY, 1.0*soundFiles.data{k}.soundFs);

end