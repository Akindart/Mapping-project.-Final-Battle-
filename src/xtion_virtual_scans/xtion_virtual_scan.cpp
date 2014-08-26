#include "ros/ros.h" 
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "scan_virtualizer.cpp"
//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Eigen
#include <Eigen/Dense>
//PCL
#include <boost/thread/thread.hpp>

using namespace Eigen;
using namespace ros;
using namespace std;

void transformCB(){

	

}


int main(int argc, char* argv[])
{

	ros::init(argc, argv, "xtion_virtual_scan");

	ros::NodeHandle *node_handler = new ros::NodeHandle();

    std::string laser_frame = "front_laser_frame";

    std::cout << ("test caraleo") << std::endl;

	Scan_virtualizer *sv = new Scan_virtualizer(10, node_handler->advertise<sensor_msgs::LaserScan>("virtual_scan", 1000), laser_frame);

	ros::Subscriber sub = node_handler->subscribe("/kinect/depth_registered/image_raw", 1000, &Scan_virtualizer::virtualScanResenderCB, sv);

	ros::spin();

	return 0;

}



