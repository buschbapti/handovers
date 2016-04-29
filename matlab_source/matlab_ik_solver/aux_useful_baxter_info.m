

function [qmid, qmax, qmin] = aux_useful_baxter_info()
% return the joint angles of baxter in degrees

    j=1;
    baxter.joint(j).min   = -9.750e+01;
    baxter.joint(j).range = 1.950e+02; 
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);

    j=2;
    baxter.joint(j).min   = -1.230e+02;
    baxter.joint(j).range = 1.830e+02; 
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
   

    j=3;
    baxter.joint(j).min   = -1.750e+02;
    baxter.joint(j).range = 3.500e+02;
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
    
    j=4;
    baxter.joint(j).min   = -2.865e+00;
    baxter.joint(j).range = 1.529e+02;
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
    
    j=5;
    baxter.joint(j).min   = -1.753e+02;
    baxter.joint(j).range = 3.505e+02;
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
    
    j=6;
    baxter.joint(j).min   = -9.000e+01;
    baxter.joint(j).range = 2.100e+02;
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
    
    j=7;
    baxter.joint(j).min   = -1.753e+02;
    baxter.joint(j).range = 3.505e+02;
    baxter.joint(j).middle = getmiddle(baxter.joint(j).min, baxter.joint(j).range);
    
    for j=1:7
        %fprintf(['joint %g. min %g, middle %g, max %g\n'], j, baxter.joint(j).min, baxter.joint(j).middle, baxter.joint(j).min+baxter.joint(j).range)
        qmid(j) = baxter.joint(j).middle;
        qmax(j) = baxter.joint(j).min+baxter.joint(j).range;
        qmin(j) = baxter.joint(j).min;
    end

end


function middle = getmiddle(min, range)

    middle = min+range/2;

end



