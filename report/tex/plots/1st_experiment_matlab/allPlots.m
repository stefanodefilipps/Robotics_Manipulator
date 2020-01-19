mainDir = '/home/marco/Scrivania/ROB2_project';
al = '/Results/control_points_always_present';
nAl = '/Results/control_points_not_always_present';
far = '/far_obstacle';
near = '/obstacle_close_to_path';
on = '/obstacle_on_path';
whereIs = {al,nAl};
whoIs = {far,near,on};

go = {'/report/tex/plots/far','/report/tex/plots/near','/report/tex/plots/on'};
to = {'/always','/notAlways'};

Titles = {'control_points_distances','ee_task_error','elbow_task_error','joint_angles','joint_velocities'};
for w=1:3
    for W=1:2
        disp([mainDir,whereIs{W},whoIs{w}])
        plots([mainDir,whereIs{W},whoIs{w}]);
        pause(0.001);
        printPdf;
        %movefile('*.eps',[mainDir,go{w},to{W}])
        for file=1:5
            movefile([Titles{file},'.eps'],[mainDir,go{w},to{W}])
        end
        disp(['to: ',mainDir,go{w},to{W}]);
        close all;
    end
end