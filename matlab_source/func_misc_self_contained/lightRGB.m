function vec = lightRGB(rgb)

    u = 0.7;
    vec = [u u u];    
    
    if rgb==(1)
        vec(1) = 1;
    end
    if rgb==(2)
        vec(2) = 1;
    end
    if rgb==(3)
        vec(3) = 1;
    end    
    
    if 0
        vecC{1} = vec;
        vecC{1}(1) = 1;

        vecC{2} = vec;    
        vecC{2}(2) = 1;

        vecC{3} = vec;    
        vecC{3}(3) = 1;
    end
end
