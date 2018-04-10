#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('robot_recognition')
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import pygame
from keras.applications.mobilenet import MobileNet
from keras.models import Model
from keras.layers import Dense, Input
from keras import backend as K

img_rows, img_cols = 224, 224
input_tensor = Input(shape=(img_rows,img_cols,3))
input_shape = (img_rows, img_cols, 3)
ImageList = ["RobotImages/image3.png", "RobotImages/image338.png", "RobotImages/image90.png", "RobotImages/image220.png", "RobotImages/image431.png"]

K.set_image_data_format('channels_last')


def cnn_model(input_shape):
    mobilenet_model = MobileNet(input_shape=(img_rows, img_cols, 3),
                                input_tensor=input_tensor,
                                pooling='avg',
                                include_top=False,
                                weights=None)
    x = mobilenet_model.output
    prediction = Dense(1, activation="sigmoid")(x)
    
    model = Model(inputs=input_tensor, outputs=prediction)

    return model
model = cnn_model(input_shape)

class NavigationNode:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot1/camera/rgb/image_raw",Image, self.imageCallback)
    self.odom_sub = rospy.Subscriber("/robot1/odom",Odometry,self.odomCallback)
    self.cmd_vel = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=1)
    self.lastLinearVelocityX = None
    self.lastAngularVelocityZ = None
    self.lastImage = None
    self.lastImageT1 = None
    self.process = False

  def odomCallback(self, data):
    #print("Linear:",data.twist.twist.linear)
    self.lastLinearVelocityX = data.twist.twist.linear.x
    #print("Angular",data.twist.twist.angular)
    self.lastAngularVelocityZ = data.twist.twist.angular.z

  def imageCallback(self,data):
    try:
      cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(data, "bgr8"), (224, 224), interpolation = cv2.INTER_CUBIC)
      self.lastImage = cv_image
      self.process = True;
    except CvBridgeError as e:
      print(e)
  def run(self):

    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    try:
      i=0
      while True:
        #rospy.spin()
        for event in pygame.event.get():
          if event.type == QUIT: sys.exit()
          if event.type == KEYDOWN and event.key == 275: # left
            move_cmd.angular.z -= 0.1
          if event.type == KEYDOWN and event.key == 276: # right
            move_cmd.angular.z += 0.1
          if event.type == KEYDOWN and event.key == 273: # straight
            move_cmd.linear.x = 0.5
          if event.type == KEYDOWN and event.key == 274: # stop
            if move_cmd.linear.x > 0:
              move_cmd.linear.x = 0.0
        if self.process:
          ImageList_ = []
          ImageList_.append(np.asarray(self.lastImage).astype('float32')/255)
          print(model.predict(np.asarray(ImageList_).reshape(1, img_rows, img_cols, 3))[0][0]*100, end='')
          print("%")
          pygame.event.pump()
          cv2.imshow("Image window", self.lastImage)
          cv2.waitKey(3)

        self.cmd_vel.publish(move_cmd)
        i+=1
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()


def main(args):
  model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
  model.load_weights('./IRW.hdf5')
  pygame.init()
  pygame.display.set_mode((100,100))

  ic = NavigationNode()
  rospy.init_node('recognizeRobot', anonymous=True)
  ic.run()
  

if __name__ == '__main__':
	main(sys.argv)