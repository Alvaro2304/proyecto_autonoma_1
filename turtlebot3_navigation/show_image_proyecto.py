#!/usr/bin/env python3
#   Este nodo se suscribe a una imagen de ROS, la convierte en una matriz de
#   OpenCV y la muestra en pantalla
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

can_detection = 0

class Cam(object):
  def __init__(self, topic_name="camera_frame"):
    self.bridge = CvBridge()
    self.image = np.zeros((10,10))
    isub = rospy.Subscriber(topic_name, Image, self.image_callback)

  def image_callback(self, img):
    self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

  def get_image(self):
    return self.image


if __name__ == '__main__':

  # Inicializar el nodo de ROS
  rospy.init_node('camera_node')

  # Objeto que se suscribe al tópico de la cámara
  topic_name = "/camera/rgb/image_raw"
  # topic_name = "/usb_cam/image_raw"
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
  # Bucle principal
  while not rospy.is_shutdown():
    
    # Obtener la imagen del tópico de ROS en formato de OpenCV
    I = cam.get_image()
    
    # Realizar algún tipo de procesamiento sobre la imagen
    
    if len(I.shape) == 3 and I.shape[2] == 3:
      img_hsv = cv2.cvtColor(I,cv2.COLOR_BGR2HSV)

      #Filtrado de color rojo
      lower1 = np.array([0, 25, 20])
      upper1 = np.array([10, 255, 255])

      lower2 = np.array([160,50,20])
      upper2 = np.array([179,255,255])

      lower_mask = cv2.inRange(img_hsv, lower1, upper1)
      upper_mask = cv2.inRange(img_hsv, lower2, upper2)

      full_mask = lower_mask + upper_mask

      #kernel de operador morfologico
      kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
      #Operador morfologico para eliminar ruido
      full_mask_2 = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)

      #Deteccion de contorno
      contours, hierarchy = cv2.findContours(full_mask_2, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      if len(contours) > 0:
        max_cont = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(max_cont)
        cv2.rectangle(I, (x,y), (x+w, y+h), (0,255,0), 1)
        can_detection = 1
      cv2.imshow('Can detector',I)
      pubflag.publish(can_detection)

    else:
      pass

    # Mostrar la imagen
    # cv2.imshow("Imagen Camara Turtlebot3", I)
    # Esperar al bucle para actualizar
    cv2.waitKey(1)
    # Opcional: publicar la imagen de salida como tópico de ROS
    #pubimg.publish(cam.bridge.cv2_to_imgmsg(I))
    rate.sleep()

cv2.destroyAllWindows()