#include "ros/ros.h" 
///C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Message
#include "least_squares_rp_project/normals_.h"

#include "aligner.cpp"

using namespace std;
using namespace Eigen;
using namespace Utils;

int main(int argc, char* argv[]){

	ros::init(argc, argv, "aligner");

	ros::NodeHandle *node_handler = new ros::NodeHandle();

	Aligner *algn = new Aligner (node_handler->advertise<least_squares_rp_project::pointAlignment>("transToAlign", 1000));

	ros::Subscriber sub = node_handler->subscribe("/normals", 1000, &Aligner::pointAlignerCB, algn);
	
	//PointAlignerTest();

	ros::spin();

	return 0;

}