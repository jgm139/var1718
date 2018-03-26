#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <fstream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/viz/vizcore.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

const int N_IMAGES = 2;
const string fileT1 = "images/traseras/t1.png";
const string fileT2 = "images/traseras/t2.png";
void imageTraseraCallback(const sensor_msgs::ImageConstPtr& t1, const sensor_msgs::ImageConstPtr& t2, const CameraInfoConstPtr& cam, const CameraInfoConstPtr& cam2){
     

    int width = 224, height = 224;

    cv::Mat m1;
    cv::resize(cv_bridge::toCvShare(t1, "mono8")->image, m1, cv::Size(width,height));

    cv::Mat m2;
    cv::resize(cv_bridge::toCvShare(t2, "mono8")->image, m2, cv::Size(width,height));


    Mat disp;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(-55,32,11,120,2000,32,0,15,1000,16,StereoSGBM::MODE_HH);
    //Ptr<StereoSGBM> sgbm = StereoSGBM::create(-10,16,11,0,4000,32,0,15,1000,16,StereoSGBM::MODE_HH);

    sgbm->compute(m1, m2, disp);
    Mat disp8;
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    
    /*imshow("final2", disp8);
    waitKey(5);*/

    float CM1f[3][3], CM2f[3][3], D1f[5], D2f[5];
    unsigned i = 0;
    unsigned j = 0;
    for(i = 0;i<3;i++){
        for(j = 0;j<3;j++){
            CM1f[i][j] = cam->R[(i*3)+j];
            CM2f[i][j] = cam2->R[(i*3)+j];
        } 
    }
    for(i = 0;i<5;i++){
        D1f[i] = cam->R[i];
        D2f[i] = cam2->R[i];
    }

   // Mat CM1(3,3,CV_32FC1, &CM1f), CM2(3,3,CV_32FC1, &CM2f), D1(1,5,CV_32FC1, &D1f), D2(1,5,CV_32FC1, &D2f);

    double r[3][3] = {{9.998381e-01, 1.610234e-02, 8.033237e-03},{-1.588968e-02, 9.995390e-01, -2.586908e-02 },{-8.446087e-03, 2.573724e-02, 9.996331e-01}};
    double t[3][4] = {{ -5.706425e-01}, {8.447320e-03}, {1.235975e-02}};

    Mat R (3,3, CV_64FC1, r);
    Mat T (3,1, CV_64FC1, t);

    //Mat R1, R2, T1, T2, Q, P1, P2;

    //stereoRectify(CM1, D1, CM2, D2, m1.size(), R, T, R1, R2, P1, P2, Q);

    float data[4][4] = {{1,  0, 0, -m1.cols/2.0},{0, -1, 0, m1.cols/2.0}
    ,{0,  0, 0, -0.8*m1.cols},{0,  0, 1, 0}};
    Mat Image3D, conversion, Q(4,4,CV_32FC1,&data);
    reprojectImageTo3D(disp, Image3D, Q);

    viz::Viz3d myWindow("Coordinate Frame");

    while (!myWindow.wasStopped())
    {
        /// Create a cloud widget
        viz::WCloud cw(Image3D, viz::Color::white());

        /// Display it in a window
        myWindow.showWidget("CloudWidget1", cw);

        myWindow.spinOnce(1, true);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_trasera");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> t1_sub(nh, "robot1/trasera1/trasera1/rgb/image_raw", 1);
    message_filters::Subscriber<Image> t2_sub(nh, "robot1/trasera2/trasera2/rgb/image_raw", 1);
    message_filters::Subscriber<CameraInfo> cam_sub(nh, "robot1/trasera1/trasera1/rgb/camera_info", 1);
    message_filters::Subscriber<CameraInfo> cam2_sub(nh, "robot1/trasera2/trasera2/rgb/camera_info", 1);

    TimeSynchronizer<Image, Image, CameraInfo, CameraInfo> sync(t1_sub, t2_sub, cam_sub, cam2_sub, 10);
    sync.registerCallback(boost::bind(&imageTraseraCallback, _1, _2, _3, _4));
    ros::Rate rate(10.0);
    while(nh.ok()){
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    ros::shutdown();
}