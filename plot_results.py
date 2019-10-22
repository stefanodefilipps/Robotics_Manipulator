import matplotlib.pyplot as plt
import math
import pickle
from mpl_toolkits.mplot3d import Axes3D
joint_angles_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/joint_angles.txt', 'r')
joint_velocities_file = open('/home/stefano/ROB2_project/Results/obstacle_on_path/velocities.txt', 'r')
joint_angles = [[],[],[],[],[],[],[]]
joint_velocities = [[],[],[],[],[],[],[]]
for line_angles,line_velocities in zip(joint_angles_file,joint_velocities_file):
    l_angles = line_angles.strip("\n").split(";")
    l_velocities = line_velocities.strip("\n").split(";")
    for i in range(len(l_angles)):
        joint_angles[i].append(float(l_angles[i]))
        joint_velocities[i].append(float(l_velocities[i]))
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