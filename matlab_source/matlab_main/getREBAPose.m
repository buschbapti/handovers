function pose = getREBAPose(dataEntry)

    rebaTable = [
        %
        % test this one, sent by email on the 23.11.2015 by Baptiste (bug!!)
        % 1.21201400166	0.520869611296	0.130807251225	0.608650259397	0.762804823551	0.209418835894	0.0617852253841
        %
        % maybe fixed
        1.212014    0.52086961    0.13080725    0.5784622    0.49569571    0.28229925    0.58307322
        %
        %
     ];
 
 
    pose = rebaTable(dataEntry,:);

end