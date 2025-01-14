#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('robot_navegation')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np
from keras.applications.mobilenet import MobileNet
from keras.models import Model
from keras.layers import Dense, Input
from keras import backend as K


image_size = 224
input_shape = (image_size, image_size, 3)
input_tensor = Input(shape=input_shape)
K.set_image_data_format('channels_last')

def cnn_model(input_shape):
    mobilenet_model = MobileNet(input_shape=input_shape,
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
    self.cmd_vel = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=1)
    self.lastImage = None
    self.process = False

  def imageCallback(self,data):
    try:
      cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(data, "rgb8"), (image_size, image_size), interpolation = cv2.INTER_CUBIC)
      self.lastImage = cv_image
      self.process = True;
    except CvBridgeError as e:
      print(e)
  def run(self):

    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    while True:
      if self.process:
        ImageList_ = []
        ImageList_.append(np.asarray(self.lastImage).astype('float32')/255)
        prediction = model.predict(np.asarray(ImageList_).reshape(1, image_size, image_size, 3))
        cv2.imshow("Image window", self.lastImage)
        cv2.waitKey(3)
        move_cmd.linear.x = move_cmd.linear.y = move_cmd.angular.z = 0;   

        if (prediction[0][0]*100) < 50:
          move_cmd.angular.z += 0.4
          move_cmd.linear.x += 0.3
        else:
          move_cmd.angular.z += -0.4
          move_cmd.linear.x += 0.3

      self.cmd_vel.publish(move_cmd)
    cv2.destroyAllWindows()

def main(args):
  model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
  model.load_weights('./AN.hdf5')

  ic = NavigationNode()
  rospy.init_node('navigationRobot', anonymous=True)
  ic.run()
  

if __name__ == '__main__':
	main(sys.argv)