import numpy  as np
import matplotlib.pyplot as plt
data = np.loadtxt('/home/utec/catkin_ws/src/turtlebot3/turtlebot3/poses_pub/pos_actual.txt')

x = data[:, 1]
y = data[:, 2]

booleano=data[:,4]
for i in range(len(booleano)):
    if booleano[i]==1:
        plt.plot(-y[i], x[i],'.r')
    else:
       plt.plot(-y[i], x[i],'.b')
plt.xlim([-3,3])
plt.ylim([4,-4])
plt.grid(True)
plt.show()