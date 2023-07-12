import numpy  as np
import matplotlib.pyplot as plt

data_can=np.loadtxt('/home/utec/catkin_ws/src/turtlebot3/turtlebot3/poses_pub/pos_can.txt')

px = data_can[:, 1]
py = data_can[:, 2]

booleano=data_can[:,4]
for i in range(len(booleano)):
    if booleano[i]==1:
        plt.plot(-py[i], px[i],'.r')

plt.xlim([-3,3])
plt.ylim([4,-4])
plt.grid(True)
plt.show()