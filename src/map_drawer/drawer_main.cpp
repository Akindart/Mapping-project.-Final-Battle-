#include "ros/ros.h" 
///C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Message

#include "least_squares_rp_project/normals_.h"

#include "least_squares_rp_project/pointAlignment.h"

#include <Eigen/Dense>

#define PI 3.14159265

using namespace Eigen;
using namespace std;

void tempCB(const least_squares_rp_project::pointAlignment msg){}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "map_drawer");

	ros::NodeHandle *node_handler = new ros::NodeHandle();

	ros::Subscriber sub = node_handler->subscribe("/transToAlign", 1000, &tempCB);
	
	ros::spin();

	return 0;

}