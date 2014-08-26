	#include "ros/ros.h" 
#include <sensor_msgs/LaserScan.h>
#include <math.h>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>
//Message
#include "least_squares_rp_project/normals_.h"

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>

#ifndef NORMALS_H
#define NORMALS_H
#endif

#define PI 3.14159265

using namespace Eigen;
using namespace std;

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > listVec2f;

class Normals;

class Normals{

	int k_neighborhood;
	
	ros::Publisher pub;
	
	listVec2f laserPoints; 

	int test;

	tf::TransformListener *listener;

	Vector2f transPoint(float x, float y, const sensor_msgs::LaserScan msg){

		tf::StampedTransform transform;

		Vector2f tempVec;

		geometry_msgs::PointStamped laser_point;
		laser_point.header.frame_id = msg.header.frame_id;

		laser_point.header.stamp = ros::Time();

		laser_point.point.x = x;
  		laser_point.point.y = y;
  		laser_point.point.z = 0.0;

  		try{
      		
      		geometry_msgs::PointStamped base_point;

      		listener->transformPoint("odom", laser_point, base_point);

      		tempVec(0,0) = base_point.point.x;
      		
      		tempVec(1,0) = base_point.point.y;
				
			return tempVec;	
   			  		
  		}	
    	
    	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
    	}

		return Vector2f(x, y);

	}

	least_squares_rp_project::normals_ *generateNormals(const sensor_msgs::LaserScan msg){

		int num_laser_beans = msg.ranges.size();

		Vector2f tempVec;

		least_squares_rp_project::normals_ *normal_msg = new least_squares_rp_project::normals_();

		normal_msg->header.seq = msg.header.seq;
		normal_msg->header.stamp = msg.header.stamp;
		normal_msg->header.frame_id = msg.header.frame_id;

		laserPoints.clear();

		//cout << "Number num_laser_beans " << num_laser_beans << endl;
		
		for(int i=0; i< num_laser_beans || (msg.angle_max - i*msg.angle_increment) < msg.angle_min; i++){

			if(msg.ranges.at(i) > 0){
				
				Vector2f tempVec;

				float x = msg.ranges.at(i)*cos(msg.angle_max - i*msg.angle_increment);
				float y = msg.ranges.at(i)*sin(msg.angle_max - i*msg.angle_increment);

				try{

					tempVec = transPoint(x, y, msg);
				
				}
				catch(tf::TransformException ex){

					tempVec(0,0) = x;
					tempVec(1,0) = y;

				}				

				laserPoints.push_back(tempVec);
				normal_msg->usefulLaserBeans.push_back(i);

			}

		}
		
		ofstream outPutFile;

    	// if(test%24 == 0){
    		
    	// 	ostringstream convert;

    	// 	convert << "/home/spades/normals/normals.dat";

    	// 	outPutFile.open(convert.str().c_str(),  std::ofstream::out | std::ofstream::app);

    	// }

		for(int i=0; i<laserPoints.size(); i++){
			
			Vector2f q_c(0,0);

			//q_c = laserPoints.at(i);
		
			for(int j=1; j < this->k_neighborhood; j++){

				if(i-j >= 0) q_c += laserPoints.at(i-j);					

				if(i+j < laserPoints.size()) q_c += laserPoints.at(i+j);	

				//cout << "loop sum vecs with i = " << i << " and j = " << j <<endl;

			}

			q_c /= (2*k_neighborhood);

			//cout << "q_c /= (2*k_neighborhood+1);" << endl;

			Vector2f tempDeltaVec;

			//tempDeltaVec = laserPoints.at(i) - q_c;

			//cout << "tempDeltaVec = laserPoints.at(i) - q_c;" << endl;

			Matrix2f covMat;

			//covMat = tempDeltaVec * tempDeltaVec.transpose();

			covMat << 0,0,0,0;

			for(int j=1; j < this->k_neighborhood; j++){

				if(i-j >= 0){

					tempDeltaVec = laserPoints.at(i-j) - q_c;
					covMat += (tempDeltaVec * tempDeltaVec.transpose());

				}
				if(i+j < laserPoints.size()){

					tempDeltaVec = laserPoints.at(i+j) - q_c;
					covMat += (tempDeltaVec * tempDeltaVec.transpose());

				}

			}

			covMat /= (2*k_neighborhood);

			SelfAdjointEigenSolver<Matrix2f> eigensolver(covMat);

			if(eigensolver.info() != Success) abort();

    		// if(test%24 == 0) 
    		// 	outPutFile << laserPoints.at(i)(0,0) << " " << laserPoints.at(i)(1,0) << " " << eigensolver.eigenvectors()(0, 0) << " " << eigensolver.eigenvectors()(1, 0) << "\n";

			normal_msg->x_components.push_back(laserPoints.at(i)(0,0));
			normal_msg->y_components.push_back(laserPoints.at(i)(1,0));

			normal_msg->x_comp_eigenvector.push_back(eigensolver.eigenvectors()(0, 0));
			normal_msg->y_comp_eigenvector.push_back(eigensolver.eigenvectors()(1, 0));
			normal_msg->eigenvalues.push_back(eigensolver.eigenvalues()(0 ,0));

		}

		// if(test%24 == 0){
		 	
		//  	outPutFile.close();	
		
		// }

		test++;

		return normal_msg;


	}	

public:

	Normals(int k_neighborhood, ros::Publisher pub){

		this->k_neighborhood = k_neighborhood;

		this->pub = pub;

		test = 0;

		listener = new tf::TransformListener();

	}

	void normalsCBPublisher(const sensor_msgs::LaserScan msg){

		least_squares_rp_project::normals_ *normal = this->generateNormals(msg);

		//cout << "least_squares_rp_project::normals_ *normal = this->generateNormals(msg);" << endl;

		pub.publish(*normal);

		//cout << "pub.publish(*normal);" << endl;

	} 

};