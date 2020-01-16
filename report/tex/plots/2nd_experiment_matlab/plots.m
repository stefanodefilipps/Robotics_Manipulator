clear all
close all
clc

mainDir = '/home/marco/Scrivania/ROB2_project';
raw = '/Results/2obs';
plotDir = '/report/tex/plots/2obs';

Titles = {'control_points_distances','control_points_distances_2','ee_task_error','elbow_task_error','joint_angles','joint_velocities'};
files = {'cps_distances.txt','cps_distances_2.txt','ee_task_error.txt','elbow_task_error.txt','joint_angles.txt','joint_velocities.txt'};
titles = {'control points distances','control points distances 2','ee task error','elbow task error','joint angles','joint velocities'};
nPlots = length(files);
switching = readmatrix(strcat(mainDir,raw,'/switching_instants.txt'));
switching = switching(:,1);

T = 0.005;
t = linspace(1,344,344)*T;
boolVal = isempty(switching);
if(~boolVal) 
    switching = switching(:,1);
end
for i = 1:nPlots
   data = readmatrix(strcat(mainDir,raw,'/',files{i}));
   y_max = max(max(data));
   y_min = min(min(data));
   h = figure(i);
   h.Renderer = 'Painters';
   [row_data,col_data] = size(data);
   for j = 1:col_data
      plot(t,data(:,j),'LineWidth',2);
      hold on;
   end
   if(~boolVal) 
       for j=1:length(switching)
       plot(T*[switching(j),switching(j)],[y_max*1.2,y_min*0.8],'Color',[0 0 0]+0.60,'LineWidth',.5);
       end
   end
   title(titles{i});
   %axis([0,row_data,y_min*0.8,y_max*1.2])
   grid on;
   xlabel('time [s]')
end

figure(1);
lg=legend('point_1','point_2','point_3');
lg.Location = 'best';
ylabel('distance [m]')

figure(2);
lg=legend('point_1','point_2','point_3');
lg.Location = 'best';
ylabel('distance [m]')

figure(3)
ylabel('error [m]');

figure(4)
ylabel('error [°]');

figure(5);
lg=legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7');
set(lg,'color','none','Location','best');
ylabel('[rad]');

figure(6);
lg=legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7');
set(lg,'color','none');
lg.Location = 'best';
ylabel('[rad/s]');

pause(0.001)

for k=nPlots:-1:1
    figure(k);
    pause;
    print(titles{k},'-depsc');
    pause(0.001);
    close(figure(k));
end

for file=1:nPlots
    movefile([titles{file},'.eps'],[mainDir,plotDir])
end
