function dataMocap = prepareMocapDataStructure(mocapNames)

    for k=1:numel(mocapNames)
        dataMocap.t=[];
        dataMocap.(mocapNames{k})=[];
    end
    
    dataMocap.names = mocapNames;
    
end