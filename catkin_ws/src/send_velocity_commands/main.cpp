#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

class RobotDriver{
  	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "/base_controller/command" topic to issue commands
		ros::Publisher cmd_vel_pub_;

  	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 1);
		}

		//! Loop forever while sending drive commands based on keyboard input
		bool driveKeyboard(){
			cout << "Type a command and then press enter.  "
				"Use 'w' to move forward, 'a' to turn left, "
				"'d' to turn right, '.' to exit.\n";

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;

			char cmd[50];
			
			while(nh_.ok()){
				cin.getline(cmd, 50);
				ros::spinOnce();

				if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='d' && cmd[0]!='.'){
					cout << "unknown command:" << cmd << "\n";
					continue;
				}

				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
				//move forward
				if(cmd[0]=='w'){
					base_cmd.linear.x += 0.4;
				} 
				//turn left (yaw) and drive forward at the same time
				else if(cmd[0]=='a'){
					base_cmd.angular.z += 0.3;
					base_cmd.linear.x += 0.4;
				} 
				//turn right (yaw) and drive forward at the same time
				else if(cmd[0]=='d'){
					base_cmd.angular.z += -0.3;
					base_cmd.linear.x += 0.4;
					
				} 
				//quit
				else if(cmd[0]=='.'){
					break;
				}

				//publish the assembled command
				cmd_vel_pub_.publish(base_cmd);
			}
			
			return true;
		}

};

int cont = 0;
uint8_t i = 0;
vector<uint8_t> v;
fstream ficheroDatos;
string datosNav = "datosNav.txt";

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg, const nav_msgs::Odometry::ConstPtr& odomMsg){
	ficheroDatos.open(datosNav.c_str(), ios::out | ios::app);

	if(ficheroDatos.is_open()){
		ficheroDatos << "images/image" << cont << ";" << odomMsg->twist.twist.linear.x;
		ficheroDatos << ";" << odomMsg->twist.twist.angular.z << endl;
		try{
			string s = static_cast<ostringstream*>( &(ostringstream() << cont) )->str();
			string fileName = "images/image" + s + ".png";
			
			cv::Mat dest;
			cv::resize(cv_bridge::toCvShare(imageMsg, "mono8")->image, dest, cv::Size(224,224));
			cv::imwrite(fileName, dest);
			cv::waitKey(30);
			
			cont++;
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imageMsg->encoding.c_str());
		}
	}
	ficheroDatos.close();
}

int main(int argc, char** argv){
	/*
	imagen;linear.x;angular.z
	*/
	ros::init(argc, argv, "robot_control");

	ros::NodeHandle nh;

	RobotDriver driver(nh);

	message_filters::Subscriber<Image> image_sub(nh, "robot1/camera/rgb/image_raw", 1000);
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "robot1/odom", 1000);

  	TimeSynchronizer<Image, nav_msgs::Odometry> sync(image_sub, odom_sub, 10);
  	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	driver.driveKeyboard();
	ros::shutdown();
}
