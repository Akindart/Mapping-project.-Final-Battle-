#include "ros/ros.h" 
//Utils
#include "../align_pack/utils.cpp"

#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace Utils;

int counter;

void writeToFile(listVec2f *set, string fileName){


	ofstream outPutFile;
	
	ostringstream convert;
		
	outPutFile.open(fileName.c_str(),  std::ofstream::out);

	for(int i=0; i<set->size(); i++) outPutFile << set->at(i)(0,0) << " " << set->at(i)(1,0) << endl;

	outPutFile.close();	

}

void tempCB(const least_squares_rp_project::pointAlignment::Ptr msg){

	listVec2f *set_1 = new listVec2f();
	listVec2f *set_2 = new listVec2f();
	listVec2f *auxListSet2 = new listVec2f();
	listVec2f *auxListSet1 = new listVec2f();

	int max_size;

	

	Vector3f transform;

	for(int i=0; i<msg->transf.size(); i++) transform(i, 0) = msg->transf.at(i);

	Matrix3f transfMatrix = v2tRad(transform);

	if(msg->x_setSeqBase.size() < msg->x_setSeqTrans.size()) max_size = msg->x_setSeqTrans.size();
	else max_size = msg->x_setSeqBase.size();

	for(int i=0; i<max_size; i++){

		try{

			Vector2f tempVec(msg->x_setSeqBase.at(i), msg->y_setSeqBase.at(i));

			auxListSet1->push_back(Vector2f(tempVec(0,0) + transform(0,0), tempVec(1,0) + transform(1,0)));

			set_1->push_back(tempVec);

		}
		catch(exception& e){

			cout << "set_1 has ended or something else --> e: " << e.what() << endl;
		}

		try{

			Vector3f tempVec(msg->x_setSeqTrans.at(i), msg->y_setSeqTrans.at(i), 1);

			auxListSet2->push_back(tempVec.head(2));

			//cout << tempVec << endl;

			//cout << transfMatrix.block(0,0,2,3) << endl;

			tempVec = transfMatrix*tempVec;

			//cout << tempVec.block(0,0,2,1) << endl;
 
			set_2->push_back(tempVec.block(0,0,2,1));
			
			//cout <<  " -----------\n " << set_2->back() << endl;

		}
		catch(exception& e){

			cout << "set_2 has ended or something else --> e: " << e.what() << endl;

		}		

	}

	if(transform(0,0) != 0 && transform(1,0) != 0 && transform(2,0) != 0 && counter ==1) {
	
		writeToFile(set_1, "/home/spades/normals/set1.dat");
		writeToFile(set_2, "/home/spades/normals/set2.dat");
		writeToFile(auxListSet1, "/home/spades/normals/set1_mod.dat");
		writeToFile(auxListSet2, "/home/spades/normals/set2_raw.dat");
		counter = 0;


	}
}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "map_drawer");

	counter = 1;

	ros::NodeHandle *node_handler = new ros::NodeHandle();

	ros::Subscriber sub = node_handler->subscribe("/transToAlign", 1000, &tempCB);
	
	ros::spin();

	return 0;

}