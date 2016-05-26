function rgb_ = gradientOfColor(nMax, type)

    switch type

        case 'gray'
            % Gradient of color. From light gray to black
            rgb_ = linspace(0.8, 0, nMax);
            %style   = struct('Color',  [(rgb_(p))  (rgb_(p)) (rgb_(p))], 'LineStyle', '-', 'LineWidth', 1.5);
            for p = 1:nMax
                style   = struct('Color',  [(rgb_(p))  (rgb_(p)) (rgb_(p))], 'LineStyle', '-', 'LineWidth', 1.5);
            end

    end
    
    
end
