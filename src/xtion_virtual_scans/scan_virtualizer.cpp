#include "ros/ros.h" 
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <math.h>
//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>
//Message
#include "least_squares_rp_project/virtual_scan.h"
#include <geometry_msgs/PointStamped.h>

#ifndef SCAN_VIRTUALIZER_H
#define SCAN_VIRTUALIZER_H
#endif

#define PI 3.14159265

using namespace Eigen;
using namespace cv_bridge;

class Scan_virtualizer;

class Scan_virtualizer{

	float angle_step;

	int k_neighborhood;

	ros::Publisher pub;

	std::vector<float> listOfPointInScan;

	cv::Mat depth_image;

	int seq;

	std::string laser_frame;

	cv_bridge::CvImageConstPtr cv_ptr;

	void generate_virtual_scan_line(){

        //cv::imshow("Depth", this->cv_ptr->image);
            //cv::waitKey(1);

		int auxJ = 0;

		int j = 220;

		for(int i=0; i<this->cv_ptr->image.cols; i++){

            float d = this->cv_ptr->image.at<u_short>(j, i);

			//ROS_INFO("depth on i %d, j %d : %f", i, j, d);

            d *= 0.001;

            //ROS_INFO("[MIN_DEPTH] - %f at row %d and col %d", d, j, i);


			float y = (float)(319.5 - i)*d/525.0;

            //if (y > 0) ROS_INFO("[Y] : %f", y);

			float single_scan = sqrt(d*d + y*y);

            //ROS_INFO("[SINGLE SCAN] : %f", single_scan);

			this->listOfPointInScan.insert(this->listOfPointInScan.begin(), single_scan);

		}



	}

	void prepare_and_send_laser_scan(const sensor_msgs::ImageConstPtr& msg){

		sensor_msgs::LaserScan virtual_laser_scan;

		virtual_laser_scan.header.seq = this->seq;

		seq++;

		virtual_laser_scan.header.stamp = msg->header.stamp;

		virtual_laser_scan.header.frame_id = this->laser_frame;

		virtual_laser_scan.angle_min = -29.0*PI/180.0;
		virtual_laser_scan.angle_max = -virtual_laser_scan.angle_min;

		virtual_laser_scan.angle_increment = angle_step;

        virtual_laser_scan.scan_time = 0;

		virtual_laser_scan.range_min = 0.04;

		virtual_laser_scan.range_max = 13.0;

		virtual_laser_scan.ranges = listOfPointInScan;

		pub.publish(virtual_laser_scan);

	}


	public:

		Scan_virtualizer(int k_neighborhood){
			
			this->k_neighborhood = k_neighborhood;
			angle_step = (58.0/640.0)*PI/180;
			seq = 0;

			this->laser_frame = "laser_frame";

  		}

		Scan_virtualizer(int k_neighborhood, ros::Publisher pub, std::string &vs_frame){
			
			this->k_neighborhood = k_neighborhood;
			angle_step = (58.0/640.0)*PI/180;
			seq = 0;
			this->laser_frame = vs_frame;
			this->pub = pub;
			cv::namedWindow("Depth");

			
			
					
  		}		

		~Scan_virtualizer(){

			cv::destroyAllWindows();

		}

		void virtualScanResenderCB(const sensor_msgs::ImageConstPtr& msg){
		

			try{

				
                //std::cout << "[IMAGE ENCODING] : " <<  msg->encoding << std::endl;

                this->cv_ptr = toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);

	    	}	
	    	catch (cv_bridge::Exception& e){
	      		
	      		ROS_ERROR("cv_bridge exception: %s", e.what());
	      		return;
	    	
	    	}

            generate_virtual_scan_line();

	    	prepare_and_send_laser_scan(msg);

	    	listOfPointInScan.clear();
	
	    }

};
