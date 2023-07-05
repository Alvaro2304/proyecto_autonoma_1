import numpy  as np
import matplotlib.pyplot as plt
data = np.loadtxt('/home/palero/catkin_ws/src/turtlebot3/turtlebot3/poses_pub/pos_actual.txt')


x = data[:, 1]
y = data[:, 2]
plt.plot(-y, x,'.r')
plt.show()