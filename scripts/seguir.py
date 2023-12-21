#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from cgi import print_arguments
import rospy
import numpy as np
import math
import cv2
import time
import cv2.aruco as aruco
import sys
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import fotogrametria

class Follower:

    def __init__(self):
        
        self.bridge = CvBridge()
        self.cv_image = None
	    #topico da camera do robo real
	    #self.image_sub = rospy.Subscriber('/v4l/camera/image_raw/compressed',
        #topico da camera do robo simulado
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                             Twist, 
                                             queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                  LaserScan, 
	 		                                    self.laser_callback)
        
        self.twist = Twist()
        self.laser_msg = LaserScan()

        self.obstaculo = False
        self.ve_vermelho = False
        self.ve_roxo = False
        self.aruco200=False
        self.aruco100=False
        self.aruco150 = False
        self.virou_esq = False
        self.virar_dir =False
        self.parado = False

        self.passou_verm = False
        self.passou_azul = False
        self.slalow = True

        self.Area_do_contorno_r = 0
        self.Area_do_contorno_b = 0
        
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1

#     #     self.lastError = 0
#     #     self.max_vel_linear = 0.2
#     #     self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

    def laser_callback(self, msg):
        self.laser_msg = msg
        frente1 = min(msg.ranges[0:15])
        frente2 = min(msg.ranges[-15:])
        frente = min(frente1,frente2)

        left_side = min(msg.ranges[30:60])
        #print("left_side: ", left_side) 
        # print(frente)
        if frente < 0.5 and self.ve_roxo == True and frente > 0:
            print("obstaculo vermelho")
            self.obstaculo = True
        else:
            self.obstaculo = False

        if frente < 0.3:
            self.parado = True
        
        
#     # def get_laser(self, pos):
#     #     return self.laser_msg.ranges[pos]

    def gira_esq(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.3
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

    def gira_dir(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = -0.3
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

    def em_frente(self):
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

    def parar(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

    

    
    def image_callback(self, msg):
        
        try:
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            cv_image2 = cv_image.copy()
            gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # lower_yellow = np.array([20, 50, 50],dtype=np.uint8)
            # upper_yellow = np.array([50, 255, 255],dtype=np.uint8)
            lower_yellow = np.array([28, 62, 165],dtype=np.uint8)
            upper_yellow = np.array([67, 143, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            img = cv_image.copy()
            img2 = cv_image.copy()
            ## if virou_esq and completou_volta1==True:
            # #    print("identificou")
            h, w, d = cv_image.shape
            search_top = 3*h//4 - 20
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            cv_image2[0:search_top, 0:w] = 0
            cv_image2[search_bot:h, 0:w] = 0
            self.w = w
            self.h = h
            img[:,(w//2)-10:,:] = 0
            img2[:,:(w//2),:] = 0
            # cv2.imshow("img", img)


            #Codigo para decidir onde virar
            if self.aruco100==False and ids==200:
                self.virou_esq = True
                self.aruco200=True
            if self.aruco100==True and ids==200:
                self.virar_dir = True
            if ids==200:
                self.aruco200=True
            if ids!=200:
                self.aruco200=False
            if ids==100:
                self.aruco200=False
                self.aruco100=True
            if ids == 150:
                self.aruco150 = True
            hsv_cut = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask_e = cv2.inRange(hsv_cut, lower_yellow, upper_yellow)
            mask_e[0:search_top-300, 0:w] = 0
            mask_e[search_bot:h, 0:w] = 0

            hsv_cut2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
            mask_d = cv2.inRange(hsv_cut2, lower_yellow, upper_yellow)
            mask_d[0:search_top-300, 0:w] = 0
            mask_d[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            E = cv2.moments(mask_e)
            D = cv2.moments(mask_d)

            
            if self.aruco200 == True and self.aruco100 == False:
                if E['m00'] > 0:
                    self.cx = int(E['m10']/E['m00'])
                    self.cy = int(E['m01']/E['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
                    # print(f"cx:{self.cx}")
            elif self.aruco200 == True and self.aruco100== True:
                if D['m00'] > 0:
                    self.cx = int(D['m10']/D['m00'])
                    self.cy = int(D['m01']/D['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
                    # print(f"cx:{self.cx}")
            elif self.aruco200 == False and self.aruco100== True and self.virou_esq==True and self.virar_dir==False:
                if E['m00'] > 0:
                    self.cx = int(E['m10']/E['m00'])
                    self.cy = int(E['m01']/E['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
                    # print(f"cx:{self.cx}")

            else:
                if M['m00'] > 0:
                    self.cx = int(M['m10']/M['m00'])
                    self.cy = int(M['m01']/M['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)

            # cv2.imshow("IMG0", mask_d)
            # cv2.imshow("IMG1", mask_e)
            # cv2.imshow("IMG2", cv_image2)
            cv2.imshow("IMG3", cv_image)            
            cv2.waitKey(1)
            #Parte Slalow 

            #Segmenta Caixa Vermelha
            lower_red = np.array([7, 160, 160],dtype=np.uint8)
            upper_red = np.array([19, 255, 255],dtype=np.uint8)
            mask_red = cv2.inRange(hsv, lower_red, upper_red)

            #   - Contorno Caixa Vermelha
            mask_red[0:(h//2), 0:w] = 0
            mask_red[search_bot:h, 0:w] = 0            
            

            contorno_red = fotogrametria.encontrar_maior_contorno(mask_red)
            x_r,y_r,w_r,h_r = cv2.boundingRect(contorno_red)
            bb_red = [x_r, y_r, x_r+w_r, y_r+h_r]
            cv2.rectangle(cv_image,(x_r,y_r),(x_r+w_r,y_r+h_r),(0,0,0),2)

            if contorno_red is not None and self.Area_do_contorno_r > 5000:
                self.ve_vermelho = True
            elif contorno_red is not None and self.Area_do_contorno_r<5000:
                self.ve_vermelho = False

            self.Area_do_contorno_r = (w_r) * (h_r)
            # print("Area do contorno: ", self.Area_do_contorno_r)
            #Segmenta Caixe Azul
            lower_blue = np.array([103, 233, 0],dtype=np.uint8)
            upper_blue = np.array([113, 255, 161],dtype=np.uint8)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            mask_blue[0:(h//2), 0:w] = 0
            mask_blue[search_bot:h, 0:w] = 0    

            #   - Contorno Caixa Azul
            contorno_blue = fotogrametria.encontrar_maior_contorno(mask_blue)
            x_b,y_b,w_b,h_b = cv2.boundingRect(contorno_blue)
            bb_blue = [x_b, y_b, x_b+w_b, y_b+h_b]
            cv2.rectangle(cv_image,(x_b,y_b),(x_b+w_b,y_b+h_b),(0,0,0),2)

            self.Area_do_contorno_b = (w_b) * (h_b)

            if self.Area_do_contorno_b > 1000:
                self.ve_roxo = True
            elif self.Area_do_contorno_b<1000:
                self.ve_roxo = False
            cv2.imshow("esquerda", mask_e)
            cv2.imshow("blue", mask_blue)
            cv2.imshow("red", mask_red)    

            aruco.drawDetectedMarkers(cv_image, corners, ids)
            # cv2.imshow("window", cv_image)
            # cv2.imshow("tela", mask)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            print('ex', e)
    
    def control(self):
        ### BEGIN CONTROL
        # print("control")
        err = self.cx - self.w/2
        #------controle P simples-------
    
        self.twist.linear.x = 0.1
        self.twist.angular.z = -float(err) / 5000
        
        # Passando para a primeira caixa
        print(f"Ve_vermelho:{self.ve_vermelho}, Ve_roxo:{self.ve_roxo}")
        # print(f"Area do azul{self.Area_do_contorno_b}. Passou:{self.passou_verm}")
        # print(f"Area do contorno vermelho: {self.Area_do_contorno_r} e azul:{self.Area_do_contorno_b}")
        if not self.passou_verm and self.slalow:
            if self.passou_azul:
                if self.ve_vermelho and self.Area_do_contorno_r > 1500:
                    self.gira_esq()
                    #print(self.Area_do_contorno_b)
                    #print(self.ve_roxo)
                else:
                    self.gira_dir()
                    if self.aruco200 == True:
                        self.passou_azul= False
                        self.passou_verm = False
                        self.slalow = False
            else:
                if self.ve_vermelho and self.Area_do_contorno_r > 1500:
                    self.gira_esq()
                    #print(self.Area_do_contorno_b)
                    if self.Area_do_contorno_b > 35000:
                        self.passou_verm = True
                        print("SAIU")
                
                else:
                    self.gira_dir()
                    if self.Area_do_contorno_b > 35000:
                        self.passou_verm = True
                        print("SAIU")
                    

        if self.passou_verm and not self.passou_azul and self.slalow:

            if self.Area_do_contorno_r > 1500 and  self.Area_do_contorno_b < 1500 and self.Area_do_contorno_b >0:
                self.gira_esq()
                print("girano esqr")

            elif not self.ve_vermelho and not self.ve_roxo:
                self.gira_esq()
                print("Girando esquerda")

            elif self.ve_roxo and not self.ve_vermelho:
                self.gira_dir()
                print("GIRANDO DIR")
            elif self.ve_vermelho and self.ve_roxo and self.Area_do_contorno_b >100:
                self.gira_dir()
                print("GIRANDO DIRRRRRRRRRRRRRRRRr")

            elif self.Area_do_contorno_r > 23000:
                rospy.sleep(1.2)
                self.em_frente()
                rospy.sleep(1)
                self.passou_azul = True
                self.passou_verm = False
                self.aruco200 = False
                print("SAIU")
            else:
                rospy.sleep(1.5)
                self.gira_esq()
                print("GIRANDO ESQ")

            print(f"Contorno vermelho:{self.Area_do_contorno_r}")
            print(f"Contorno azul:{self.Area_do_contorno_b}")
            print(self.ve_roxo)

        if self.aruco150:
            if self.parado:
                self.parar()

        ### END CONTROL
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        # rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()

# Main loop
if __name__=="__main__":
    rospy.init_node('follower')
    follower = Follower()

    while not rospy.is_shutdown():
        follower.control()

# END ALL