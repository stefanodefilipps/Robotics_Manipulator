function plots(dir)
switching = readmatrix([dir,'/switching_instants.txt']);
% TOBE removed:
switching;% = switching(:,1);
boolVal = isempty(switching);
if(~boolVal) 
    switching = switching(:,1);
end
files = {'cps_distances.txt','ee_task_error.txt','elbow_task_error.txt','joint_angles.txt','joint_velocities.txt'};
titles = {'control points distances','ee task error','elbow task error','joint angles','joint velocities'};
T = 0.005;
t = linspace(1,152,152)*T;
for i = 1:5
   data = readmatrix([dir,'/',files{i}]);
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

figure(2)
ylabel('error [m]');

figure(3)
ylabel('error [°]');

figure(4);
lg=legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7');
set(lg,'color','none','Location','best');
ylabel('[rad]');

figure(5);
lg=legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7');
set(lg,'color','none');
lg.Location = 'best';
ylabel('[rad/s]');
end