function report_plots(experiment_name) 
    J = readmatrix(['../../../Results/',experiment_name,'/velocities.txt']);
    figure();
    [~,col] = size(J);
    for i = 1:col
        plot(J(:,i));
        hold on;
    end
    legend('first joint','second joint','third joint','fourth joint','fifth joint','sixth joint','seventh joint');
end