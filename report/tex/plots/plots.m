clear;
close;
clc;
dir = input('working directory: ','s');
switching = readmatrix([dir,'/switching_instants.txt']);
% TOBE removed:
switching = switching(:,1);
files = {'cps_distances.txt','ee_task_error.txt','elbow_task_error.txt','joint_angles.txt','joint_velocities.txt'};

for i = 1:5
   data = readmatrix([dir,'/',files{i}]);
   y_max = max(max(data));
   y_min = min(min(data));
   h = figure(i);
   [row_data,col_data] = size(data);
   for j = 1:col_data
      plot(data(:,j),'LineWidth',2);
      hold on;
   end
   plot([switching(:),switching(:)],[y_max*1.2,y_min*0.8],'Color',[0 0 0]+0.70,'LineWidth',.5,);
   title(files{i});
   axis([0,row_data,y_min*0.8,y_max*1.2])
end