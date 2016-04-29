
function TendEff = create_endeffector_positions_insertion_toy()

    TendEff = [];
    if 0
        Thandover = robot.readEntityCoordinate('handoverPosition');
        Thandover = robot.readEntityCoordinate('Dummy_tip');
        save('testHandover.mat', 'Thandover')
    else
        load('testHandover.mat');
    end

    if 1 % Perturb initial guess position
        %Thandover(1,4) = Thandover(1,4)+0.5;
        Thandover(2,4) = Thandover(2,4)+0.25;
        %Thandover(3,4) = Thandover(3,4)-0.250;
    end
    %Thandover= Thandover*se3(0, d2r(-90), 0, 0, 0, 0);
    
    TendEff{3} = Thandover;
    
    TendEff{2} = TendEff{3}*se3(0, 0, -(2*pi)/5, 0, 0, 0);
    TendEff{1} = TendEff{3}*se3(0, 0, -2*(2*pi)/5, 0, 0, 0);
    TendEff{4} = TendEff{3}*se3(0, 0, (2*pi)/5, 0, 0, 0);
    TendEff{5} = TendEff{3}*se3(0, 0, 2*(2*pi)/5, 0, 0, 0);
    

    

end