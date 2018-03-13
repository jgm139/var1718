#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

class RobotDriver{
  	private:
		//! The node handle we'll be using
		ros::NodeHandle nh_;
		//! We will be publishing to the "/base_controller/command" topic to issue commands
		ros::Publisher cmd_vel_pub_;
		fstream ficheroDatos;
		string datosNav;
		int index;

  	public:
		//! ROS node initialization
		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			//set up the publisher for the cmd_vel topic
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 1);
			datosNav = "datosNav.txt";
			index = 0;
		}

		//! Loop forever while sending drive commands based on keyboard input
		bool driveKeyboard(){
			cout << "Type a command and then press enter.  "
				"Use '+' to move forward, 'l' to turn left, "
				"'r' to turn right, '.' to exit.\n";

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;

			char cmd[50];
			ficheroDatos.open(datosNav.c_str(), ios::out | ios::app);

			if(ficheroDatos.is_open()){
				while(nh_.ok()){
					cin.getline(cmd, 50);
					string s = static_cast<ostringstream*>( &(ostringstream() << index) )->str();

					if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.'){
						cout << "unknown command:" << cmd << "\n";
						continue;
					}

					base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
					//move forward
					if(cmd[0]=='+'){
						base_cmd.linear.x = 0.75;
						ficheroDatos << "images/image" << s << ".png;0.75;" << endl;
						ros::spinOnce();
					} 
					//turn left (yaw) and drive forward at the same time
					else if(cmd[0]=='l'){
						base_cmd.angular.z = 0.75;
						base_cmd.linear.x = 0.05;
						ficheroDatos << "images/image" << s << ".png;0.05;0.75" << endl;
						ros::spinOnce();
					} 
					//turn right (yaw) and drive forward at the same time
					else if(cmd[0]=='r'){
						base_cmd.angular.z = -0.75;
						base_cmd.linear.x = 0.05;
						ficheroDatos << "images/image" << s << ".png;-0.05;0.75" << endl;
						ros::spinOnce();
					} 
					//quit
					else if(cmd[0]=='.'){
						break;
					}

					index++;

					//publish the assembled command
					cmd_vel_pub_.publish(base_cmd);
				}
			}
		
			ficheroDatos.close();
			return true;
		}

};

int cont = 0;
uint8_t i = 0;
vector<uint8_t> v;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  	try{
		string s = static_cast<ostringstream*>( &(ostringstream() << cont) )->str();
		string fileName = "images/image" + s;
		fileName += ".png";
		
		cv::Mat dest;
		cv::resize(cv_bridge::toCvShare(msg, "bgr8")->image, dest, cv::Size(224,224));
		cv::imwrite(fileName, dest);
		cv::waitKey(30);
		
		cont++;
  	}
  	catch (cv_bridge::Exception& e){
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}


int main(int argc, char** argv){
	/*
	imagen;linear.x;angular.z
	*/
	ros::init(argc, argv, "robot_driver");
	ros::init(argc, argv, "image_listener");

	ros::NodeHandle nh1, nh2;

	RobotDriver driver(nh1);
	image_transport::ImageTransport it(nh2);
	image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);

	driver.driveKeyboard();
	ros::shutdown();
}
