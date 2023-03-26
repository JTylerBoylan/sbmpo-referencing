clc
clear
close all
%%

size = 5;
res = 100;
[X,Y] = meshgrid(linspace(-size, size, res), linspace(-size, size, res));

h = sqrt(X.^2 + Y.^2);

figure
contourf(X,Y,h)

dhdx = 2*X ./ sqrt(X.^2 + Y.^2);
dhdy = 2*Y ./ sqrt(X.^2 + Y.^2);
figure
contourf(X,Y,dhdx+dhdy)