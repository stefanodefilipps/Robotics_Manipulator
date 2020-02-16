clear;
close all;
clc;

p0 = [0;0];
p1 = [-9;0];
p2 = [6,6];
h = figure(1);
h.Renderer = 'Painters';
vectarrow(p0,p1,'b')
hold on
vectarrow(p0,p2,'r')
hold on
plot([0,6],[-1,-1],'g')
legend('distance from the obstacle','repulsive velocity','projected speed')
plot([6,6],[6,-1],'--k','HandleVisibility','off')
plot([0,0],[0,-1],'--k','HandleVisibility','off')
axis([-10,7,-15,15])