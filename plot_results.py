import matplotlib.pyplot as plt
import math
import pickle
from mpl_toolkits.mplot3d import Axes3D
joint_angles_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/joint_angles.txt', 'r')
joint_velocities_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/velocities.txt', 'r')
ee_task_error_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/ee_task_error.txt', 'r')
elbow_task_error_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/elbow_task_error.txt', 'r')
joint_angles = [[],[],[],[],[],[],[]]
joint_velocities = [[],[],[],[],[],[],[]]
ee_task_error = []
elbow_task_error = []
for line_angles,line_velocities in zip(joint_angles_file,joint_velocities_file):
    l_angles = line_angles.strip("\n").split(";")
    l_velocities = line_velocities.strip("\n").split(";")
    for i in range(len(l_angles)):
        joint_angles[i].append(float(l_angles[i]))
        joint_velocities[i].append(float(l_velocities[i]))
for line_ee,line_elbow in zip(ee_task_error_file,elbow_task_error_file):
    ee_error = line_ee.strip("\n")
    elbow_error = line_elbow.strip("\n")
    ee_task_error.append(float(ee_error))
    elbow_task_error.append(float(elbow_error))


handles = []
for i in range(len(joint_angles)):
    label_joint = "joint "+str(i+1);
    h, = plt.plot(range(len(joint_angles[i])),joint_angles[i], label = label_joint)
    handles.append(h)
plt.legend(handles=handles, loc='lower right')
plt.show()
handles = []
for i in range(len(joint_velocities)):
    label_joint = "joint "+str(i+1);
    h, = plt.plot(range(len(joint_velocities[i])),joint_velocities[i], label = label_joint)
    handles.append(h)
plt.legend(handles=handles, loc='lower right')
plt.show()
plt.plot(range(len(ee_task_error)),ee_task_error)
plt.show()
plt.plot(range(len(elbow_task_error)),elbow_task_error)
plt.show()