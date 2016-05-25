
k=1; j=1; i=1;

try
    load('test.mat');
catch
    test = [];
end

reloaded = 0;
% generalize
while k <= 10
    while j <= 10
        
        if ~isempty(test) && reloaded == 0
            j = test.idx(1); k = test.idx(2);
            reloaded = 1;
        end
        
        
        
        test.idx = [j k];

        save('test.mat', 'test');
        
        j=j+1;
    end
    k=k+1;
end