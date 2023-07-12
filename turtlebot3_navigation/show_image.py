#!/usr/bin/env python3
#   Este nodo se suscribe a una imagen de ROS, la convierte en una matriz de
#   OpenCV y la muestra en pantalla
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

can_detection = 0

class Cam(object):
  def __init__(self, topic_name="camera_frame"):
    self.bridge = CvBridge()
    self.image = np.zeros((10,10))
    self.isub = rospy.Subscriber(topic_name, Image, self.image_callback)

  def image_callback(self, img):
    self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

  def get_image(self):
    return self.image


if __name__ == '__main__':

  # Inicializar el nodo de ROS
  rospy.init_node('camera_node')

  # Objeto que se suscribe al tópico de la cámara
  topic_name = "/camera/rgb/image_raw"
  cam = Cam(topic_name)

  # Tópico para publicar una imagen de salida
  topic_pub = 'image_out'
  pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

  # Topico que publica flag de deteccion
  topic_flag = 'detection'
  pubflag = rospy.Publisher(topic_flag,Bool,queue_size=1)

  # Frecuencia del bucle principal
  freq = 10
  rate = rospy.Rate(freq)
  flag_send= 0
  flag_send_last=0
  can_detection=0
  print(flag_send)
  print(flag_send_last)
  print(can_detection)
  # Bucle principal
  while not rospy.is_shutdown():
    
    # Obtener la imagen del tópico de ROS en formato de OpenCV
    I = cam.get_image()
    # Realizar algún tipo de procesamiento sobre la imagen
    if I.ndim==3:
      hsv=cv2.cvtColor(I,cv2.COLOR_BGR2HSV)
      #cv2.imshow("HSV",hsv)
      lower = np.array([160,100,20])
      upper = np.array([179,255,255])

      mask = cv2.inRange(hsv, lower, upper)
      kernel = np.ones((5,5),np.uint8)
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
      mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
      #cv2.imshow("MASK",mask)
      cont,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      #cont_img=cv2.drawContours(I,cont,-1,255,3)
      #cv2.imshow("Bordes",cont_img)
      print("-------------------------------------")
      print("antes",flag_send,",",flag_send_last)
      
      if len(cont)>0:
        c=max(cont,key=cv2.contourArea,default=0)
        x,y,h,w=cv2.boundingRect(c)
        cv2.rectangle(I,(x,y),(x+w,y+w),(0,255,0),5)
        flag_send=1        
      else:
        flag_send=0
        
    
      print("despues",flag_send,",",flag_send_last)
      if(flag_send==1):
          if(flag_send_last==0): 
            can_detection = 1
          else:
            can_detection=0
      else: 
        can_detection=0
      print("can",can_detection)
    
     
      flag_send_last=flag_send
      print("cambio",flag_send,",",flag_send_last)
      #cv2.imshow("GAA", I)
      
      # segmented_img = cv2.bitwise_and(I, I, mask=mask)
      # cv2.imshow("Segmented_image",segmented_img)
      
    pubflag.publish(can_detection)
      
    # Mostrar la imagen
    #cv2.imshow("Imagen Camara Turtlebot3", I)
    

    # Esperar al bucle para actualizar
    cv2.waitKey(1)
    # Opcional: publicar la imagen de salida como tópico de ROS
    pubimg.publish(cam.bridge.cv2_to_imgmsg(I))
    rate.sleep()

cv2.destroyAllWindows()
