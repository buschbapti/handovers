function h = figurewe(varargin)
 % h = figurew(varargin)
 % h = figurew([])
 % h = figurew
 % 
 
 
    if isempty(varargin)
        h = figure;
    else
        h = figure('name', varargin{1}, 'NumberTitle', 'off');
    end
    set(gcf, 'color', 'white');
    hold on;
    grid on;
    box on;
    
    axis 'equal';
    
end