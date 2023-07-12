import numpy  as np
import matplotlib.pyplot as plt

data_can=np.loadtxt('/home/user/catkin_ws/src/turtlebot3/turtlebot3/poses_pub/pos_can.txt')

px = data_can[:, 1]
py = data_can[:, 2]

booleano=data_can[:,4]
for i in range(len(booleano)):
    if booleano[i]==1:
        plt.plot(-py[i], px[i],'.r')


plt.show()