%% files and directories
clear all
close all
clc

mainDir = '/home/marco/Scrivania/ROB2_project';
raw = '/Results/2obs';
plotDir = '/report/tex/plots/2obs';

files = {'cps_distances.txt','cps_distances_2.txt'};
titles = {'control points distances','control points distances 2'};
Titles = {'control_points_distances','control_points_distances_2'};
%% reading datas
switching = readmatrix(strcat(mainDir,raw,'/switching_instants.txt'));
switching = switching(:,1);
obst1 = readmatrix(strcat(mainDir,raw,'/',files{1}));
obst2 = readmatrix(strcat(mainDir,raw,'/',files{2}));

%% compute minimum distances
for i = 1:3
    obstMin(:,i) = min(obst1(:,i),obst2(:,i));
end

%% parameters
d = 0.4;
D = 0.2;
T = 0.005;
t = linspace(1,344,344)*T;
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

print('min_2','-depsc');
movefile(['min_2','.eps'],[mainDir,plotDir]);