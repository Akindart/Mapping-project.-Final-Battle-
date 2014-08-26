#include "ros/ros.h" 
///C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
//Message
#include "least_squares_rp_project/normals_.h"

#include "normals.cpp"

int main(int argc, char* argv[]){

	ros::init(argc, argv, "point_aligner");

	ros::NodeHandle *node_handler = new ros::NodeHandle();

	std::cout << ("test caraleo") << std::endl;

	Normals *nrmls = new Normals (10, node_handler->advertise<least_squares_rp_project::normals_>("normals", 1000));

	ros::Subscriber sub = node_handler->subscribe("/virtual_scan", 1000, &Normals::normalsCBPublisher, nrmls);
	
	ros::spin();

	return 0;

}