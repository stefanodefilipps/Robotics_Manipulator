title = {'control_points_distances','ee_task_error','elbow_task_error','joint_angles','joint_velocities'};
for k=5:-1:1
    figure(k);
    print(title{k},'-depsc');
    pause(0.001);
    close(figure(k));
end