import matplotlib.pyplot as plt
import math
import pickle
ee_position = open('/home/stefano/Robotic2_project/plot_result/data_ee_file.txt', 'r')
joint = open('/home/stefano/Robotic2_project/plot_result/data_joints_file.txt', 'r')
ee_x = []
ee_y = []
sum_2 = []
joint_values = [[],[],[],[],[]]
for line_ee,line_joint in zip(ee_position,joint):
    l_ee = line_ee.strip("\n").split(";")
    l_joint = line_joint.strip("\n").split(";")
    ee_x.append(float(l_ee[0]))
    ee_y.append(float(l_ee[1]))
    sum_2.append(float(l_joint[0]) + float(l_joint[1]))
    for i in range(len(l_joint) - 1):
        joint_values[i].append(float(l_joint[i]))
fig = plt.figure()
plt.plot(ee_x,ee_y)
fig.suptitle('END EFFECTOR POSITION', fontsize=20)
plt.show()
plt.plot(range(len(sum_2)),sum_2)
plt.show()
'''for i in range(len(joint_values)):
    plt.plot(range(len(joint_values[i])),joint_values[i])
    plt.show()'''