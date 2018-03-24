#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

void imageTraseraCallback(const sensor_msgs::ImageConstPtr& t1, const sensor_msgs::ImageConstPtr& t2, const CameraInfoConstPtr& cam){

    string fileT1 = "images/traseras/t1.png";
    string fileT2 = "images/traseras/t2.png";
    
    cv::Mat m1;
    cv::resize(cv_bridge::toCvShare(t1, "mono8")->image, m1, cv::Size(224,224));
    cv::imwrite(fileT1, m1);
    cv::waitKey(30);

    cv::Mat m2;
    cv::resize(cv_bridge::toCvShare(t2, "mono8")->image, m2, cv::Size(224,224));
    cv::imwrite(fileT2, m2);
    cv::waitKey(30);

    cout << cam << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_trasera");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> t1_sub(nh, "robot1/trasera1/trasera1/rgb/image_raw", 1);
	message_filters::Subscriber<Image> t2_sub(nh, "robot1/trasera2/trasera2/rgb/image_raw", 1);
    message_filters::Subscriber<CameraInfo> cam_sub(nh, "robot1/trasera1/trasera1/rgb/camera_info", 1);

  	TimeSynchronizer<Image, Image, CameraInfo> sync(t1_sub, t2_sub, 10);
  	sync.registerCallback(boost::bind(&imageTraseraCallback, _1, _2, _3));

    ros::Rate rate(10.0);
    while(nh.ok()){
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    ros::shutdown();
}