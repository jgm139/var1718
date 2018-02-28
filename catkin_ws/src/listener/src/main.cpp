#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

int cont = 0;
uint8_t i = 0;
vector<uint8_t> v;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ofstream off;
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
		string s = "";
    ostringstream convert;
    convert << cont;
    string fileName = "images/image"+convert.str();
    fileName += ".txt";

    ofstream ficheroSalida;
     
    ficheroSalida.open(fileName.c_str());
		ficheroSalida.close();
		
    v = msg->data;

    cout << "Width: " << msg->width << endl;
    cout << "Height: " << msg->height << endl;

    cout << v.size();
    /*for(i=0;i<v.size();i++){
      cout << v[i] << ",";
    }*/
		 
		ficheroSalida << s << endl;
		
		cout << "Fichero creado: ";
		cout << cont << endl;
		cont++;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);

	ros::Rate rate(10.0);
	while(nh.ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

  ros::spin();
  ros::shutdown();
  cv::destroyWindow("view");
}
