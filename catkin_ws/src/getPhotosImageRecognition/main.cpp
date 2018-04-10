#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace sensor_msgs;


int lastChar = 0;
bool save = false;
bool robot = false;
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
			cout << "Type a command.  "
				"IMPORTANT: Use 'p' to take photos to the other robot, 'a' to turn left, "
				"'d' to turn right, '.' to exit.\n";

			//we will be sending commands of type "twist"
			geometry_msgs::Twist base_cmd;

			char cmd = 0;

			system ("/bin/stty raw");
			while(nh_.ok()){
				cmd = getchar(); 
				if(cmd!='w' && cmd!='a' && cmd!='d' && cmd!='.' && cmd!='r' && cmd!='p' && cmd!='n'){
					cout << "unknown command:" << cmd << "\n";
					continue;
				}

				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
				//move forward
				if(cmd=='w'){
					base_cmd.linear.x += 0.4;
					lastChar = 1;
				} 
				//turn left (yaw) and drive forward at the same time
				else if(cmd=='a'){
					base_cmd.angular.z += 0.3;
					base_cmd.linear.x += 0.4;
					lastChar = 2;
				} 
				//turn right (yaw) and drive forward at the same time
				else if(cmd=='d'){
					base_cmd.angular.z += -0.3;
					base_cmd.linear.x += 0.4;
					lastChar = 3;
				}
				//stop/start record
				else if (cmd == 'r'){
					save = !save;
				}
				else if (cmd == 'p'){
					robot = !robot;
				}
				//exit
				else if(cmd=='.'){
					break;
				}
				ros::spinOnce();
				//publish the assembled command
				cmd_vel_pub_.publish(base_cmd);
			}
			system ("/bin/stty cooked");
			return true;
		}

};

int cont = 0;
uint8_t i = 0;
vector<uint8_t> v;
fstream ficheroDatos;
string datosNav = "datosNav.txt";

void imageCallback(const sensor_msgs::ImageConstPtr& imageMsg){
	ficheroDatos.open(datosNav.c_str(), ios::out | ios::app);

	if(ficheroDatos.is_open()){
		if(save){
			save = false;
			ficheroDatos << "RobotImages2/image" << cont << ".png;" << robot << endl;
			try{
				string s = static_cast<ostringstream*>( &(ostringstream() << cont) )->str();
				string fileName = "RobotImages2/image" + s + ".png";
				
				cv::Mat dest;
				cv::resize(cv_bridge::toCvShare(imageMsg, "mono8")->image, dest, cv::Size(224,224));
				cv::imshow("Vista Conductor Nuevo", dest);
				cv::imwrite(fileName, dest);
				cv::waitKey(30);
				
				cont++;
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imageMsg->encoding.c_str());
			}
		}else{
			cv::imshow("Vista Conductor", cv_bridge::toCvShare(imageMsg, "mono8")->image);
			cv::waitKey(30);
		}
	}
	ficheroDatos.close();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "getPhotosRecognition");

	ros::NodeHandle nh;

	RobotDriver driver(nh);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("robot1/camera/rgb/image_raw", 1, imageCallback);

	driver.driveKeyboard();
	ros::shutdown();
}
