#include "ros/ros.h" 
///C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Message

#include "least_squares_rp_project/normals_.h"

#include "least_squares_rp_project/Utils.cpp"

using namespace std;
using namespace Eigen;
using namespace Utils;

void tempCB(){}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "map_drawer");

	ros::NodeHandle *node_handler = new ros::NodeHandle();

	ros::Subscriber sub = node_handler->subscribe("/transToAlign", 1000, &tempCB);
	
	ros::spin();

	return 0;

}