

function [xyz, tableHeight] = get_table_coordinates()


    table= [ -0.0  1; -0.5  0.5; -0.4750  -0.4750 ];

        cx = table(1,:);    cy = table(2,:);    cz = table(3,:);

        xyz = [ cx(1)    cy(1)  cz(1);
                cx(2)    cy(1)  cz(1);
                cx(2)    cy(2)  cz(1);
                cx(1)    cy(2)  cz(1);
                cx(1)    cy(1)  cz(1) ];
            
        tableHeight = cz(1);
            
end