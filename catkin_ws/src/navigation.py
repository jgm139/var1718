import numpy as np
import rospy
import sys
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from keras.applications.mobilenet import MobileNet

class Navigation:
    
    def __init__(self):
        self.imageSub = message_filters.Subscriber("robot1/camera/rgb/image_raw", Image)
        self.odometrySub = message_filters.Subscriber("robot1/odom", Odometry)
        self.cmd_vel = rospy.Publisher("/robot1/mobile_base/commands/velocity",Twist,queue_size=1)
        
        ts = message_filters.TimeSynchronizer([self.imageSub, self.odometrySub], 10)
        ts.registerCallback(self.callback)
        
        self.cv = CvBridge()
        self.lastImage = None
        self.OdomLX = None
        self.OdomAZ = None
        
        self.model = MobileNet()
        self.model.load_weights("./weights/WeightsMobileNetTrain1/results/weights.59-0.01.hdf5")
        self.model.compile(loss='mse',optimizer='adam',metrics=['accuracy'])

    
    def callback(self, imageMsg, odometryMsg):
        try:
            cv = self.bridge.imgmsg_to_cv2(imageMsg, "mono8")
            image = cv
            self.lastImage = cv.resize(image, (224, 224))
            self.OdomLX = odometryMsg.twist.twist.linear.x;
            self.OdomAZ = odometryMsg.twist.twist.angular.z;
        except Exception as e:
            print(e)
            
            
    def run(self):
        batch = [self.lastImage]
        inputModel = np.array(batch)
        output = self.model.predict(inputModel,batch_size=1,verbose = 0)
        move_cmd = Twist()
        move_cmd.linear.x = output[0][0]
        move_cmd.angular.z = output[0][1]
        
    
def main(args):
    node = Navigation()
    rospy.init_node("Navigation", anonymous="True")
    node.run()
    
if __name__ == '__main__':
    main(sys.argv)    
    
