% make a uicontrol that will be used to interrupt some loop
g = 0;
f = figure;
b = uicontrol('style','push','string','g++','callback','g=g+1');

% now the loop
while g < 10
  fprintf(1,'The variable g is now %i\n', g);
  drawnow
end