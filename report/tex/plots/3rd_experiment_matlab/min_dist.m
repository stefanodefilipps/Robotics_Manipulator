%% files and directories
clear all
close all
clc

mainDir = '/home/marco/Scrivania/ROB2_project';
raw = '/Results/3obs';
plotDir = '/report/tex/plots/3obs';

files = {'cps_distances.txt','cps_distances_2.txt','cps_distances_3.txt'};
titles = {'control points distances','control points distances 2','control points distances 3'};
Titles = {'control_points_distances','control_points_distances_2','control_points_distances_3'};
%% reading datas
switching = readmatrix(strcat(mainDir,raw,'/switching_instants.txt'));
switching = switching(:,1);
obst1 = readmatrix(strcat(mainDir,raw,'/',files{1}));
[l,~] = size(obst1);
obst2 = readmatrix(strcat(mainDir,raw,'/',files{2}));
obst3 = readmatrix(strcat(mainDir,raw,'/',files{3}));
%% compute minimum distances
for i = 1:3
    obstMin(:,i) = min(obst1(:,i),min(obst2(:,i),obst3(:,i)));
end

%% parameters
d = 0.4;
D = 0.2;
T = 0.005;
t = linspace(1,l,l)*T;
%% figure
h = figure(1);
h.Renderer = 'Painters';
for i = 1:3
    plot(t,obstMin(:,i),'Linewidth',2);
    hold on;
end
plot([t(1),t(end)],[d,d],'-g');
grid on;
plot([t(1),t(end)],[D,D],'-r');
yMax = max(max(obstMin));
yMin = min(min(obstMin));
for i = 1:length(switching)
    s = switching(i);
    plot([s,s]*T,[yMax*1.2,yMin*0.8],'Color',[0 0 0]+0.60,'LineWidth',.5);
end

%colors = {'-b','-r','-y'};

lg=legend('point_1','point_2','point_3');
lg.Location = 'best';
ylabel('distance [m]');
xlabel('time [s]');
title('minimum distance from the obstacles');

print('min_3','-depsc');
movefile(['min_3','.eps'],[mainDir,plotDir]);