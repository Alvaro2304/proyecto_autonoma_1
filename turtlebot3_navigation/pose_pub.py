#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Bool

class Pose_pub(object):
    def __init__(self):
       # Crear el suscriptor al tópico del LiDAR
        topic = "amcl_pose"
        self.pub = rospy.Subscriber(topic,PoseWithCovarianceStamped,self.callback)
        self.position = PoseWithCovarianceStamped()
    
    def callback(self, msg):
        # Callback para el suscriptor
        self.position = msg
        
    def get_pos(self):
        return self.position.pose.pose.position.x,self.position.pose.pose.position.y,self.position.pose.pose.position.z    

class Detection(object):
    def __init__(self):
        topic_sub = 'detection' 
        self.fsub = rospy.Subscriber(topic_sub,Bool,self.callback)
        self.detection=Bool()
        #self.can_detection = Bool()
        #rospy.sleep(1)
    

    def callback(self,msg):
        self.detection = msg
    
    def get_detection(self):
        return self.detection

if __name__ == "__main__":
   rospy.init_node("Pose_publisher")
   #Cambiar segun direccion
   
   fxact = open("/home/palero/catkin_ws/src/turtlebot3/turtlebot3/poses_pub/pos_actual.txt", "w")
   topic = 'Rob_Position'
   
   pub = rospy.Publisher(topic,String,queue_size=10)
   
   pose= Pose_pub()

   detection = Detection()

   dt = 0.1
   # Tiempo de ejecución del bucle (en Hz)
   rate = rospy.Rate(1/dt)  
   
   t = 0
   while not rospy.is_shutdown():
    px,py,pz = pose.get_pos()
    detect = detection.detection.data
    if detect:
        fxact.write(str(t)+' '+str(px)+' '+str(py)+' '+str(pz)+' '+str(1)+'\n')
#    else:
#        fxact.write(str(t)+' '+str(px)+' '+str(py)+' '+str(pz)+' '+str(0)+'\n')
    # detect = detection.detection
    # if detect:
    #    fxact.write(str(t)+' '+str(px)+' '+str(py)+' '+str(pz)+'\n')
    #    break
    print(detect)
    t+=dt
    rate.sleep()

fxact.close()