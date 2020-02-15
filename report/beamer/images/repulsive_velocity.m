clear;
close all;
clc;
a = 6;
p = 1;
v = @(x) 1./(1+exp((x*2/p - 1)*a));

x = 0:0.01:1;
a = 6;
p = 0.4;
h = figure(1);
h.Renderer = 'Painters';
plot(x,v(x));
grid on;
title('Repulsive magnitude');
ylabel('Repulse magnitude [% of Vmax]');
xlabel('Distance [% of critic distance]');
print('repulsive_velocity','-dpdf')