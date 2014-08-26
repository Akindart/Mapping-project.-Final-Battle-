#include "utils.cpp"

#include <fstream>
#include <iostream>


#ifndef ALIGNER_H
#define ALIGNER_H
#endif

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > listVec2f;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >::iterator listVec2fIterator;

class Aligner;

class Aligner{

	MatrixXf *p_set1;

	std::vector<float> eingenValues1;

	MatrixXf *p_set2;

	std::vector<float> eingenValues2;

	ros::Publisher pub;

	bool set1_free;

	MatrixXf *comparisonSet;

	listVec2f setEigenVec1;

	listVec2f setEigenVec2;

	listVec2f usedVecs;

	MatrixXf *extractPointInfo(const least_squares_rp_project::normals_ msg,  std::vector<float> *eingenValues, listVec2f *tempVecList){

		eingenValues->clear();
		tempVecList->clear();

		int iterations = msg.usefulLaserBeans.size();

		MatrixXf *p_set = new MatrixXf(2, iterations);

		Vector2f tempVec;

		for(int i=0; i<iterations; i++){

			(*p_set)(0, i) = msg.x_components.at(i);
			(*p_set)(1, i) = msg.y_components.at(i);

			tempVec(0, 0) = msg.x_comp_eigenvector.at(i);
			tempVec(1, 0) = msg.y_comp_eigenvector.at(i);

			tempVecList->push_back(tempVec);

			eingenValues->push_back(msg.eigenvalues.at(i));
			
		}

		return p_set;

	}

	void findRegionToCompare(){

		int compSetCounter = 0;

		float minDist = -1;
		int posInVec;


		for(int i=0; i<p_set1->cols(); i++){

			float x1, y1;
			x1 = (*p_set1)(0, i);
			y1 = (*p_set1)(1, i);

			//ROS_ERROR("set 1");

			if(!usedVecs.empty()) usedVecs.clear(); 

			for(int j=0; j<p_set2->cols(); j++){

				minDist = -1;

				float x2, y2;
				x2 = (*p_set2)(0, j);
				y2 = (*p_set2)(1, j);

				//ROS_ERROR("set 2");

				if(sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) < 0.3){

					if(minDist == -1){

						minDist = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
						posInVec = j;

					} 

					else if(sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) < minDist){ //here Ill put the verification for the fact that if the point of set_2 is already
																						//paired with some other point of set_1
					 	minDist = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
						posInVec = j;

					}

					//cout << "two possible equal points: (" << x1 << ", " << y1 << ") - " << "(" << x2<< ", " << y2 << ")" << endl;

				}

			}

			if(posInVec > -1 && posInVec < p_set2->cols()){
			
				//ROS_INFO("tetetetetetetet -- %d \\ Eigen vec cols: %d \\  p-set1 size: %d", i, setEigenVec1.size(), p_set1->cols());

				//cout << "setEigenVec1.at(i): \n" << setEigenVec1.at(i) << endl;

				float cos_ = setEigenVec1.at(i).dot(setEigenVec2.at(posInVec))/ (sqrt(setEigenVec1.at(i)(0,0)*setEigenVec1.at(i)(0,0) + setEigenVec1.at(i)(1,0)*setEigenVec1.at(i)(1,0) )*
																							sqrt(setEigenVec2.at(posInVec)(0,0)*setEigenVec2.at(posInVec)(0,0) + 
																								setEigenVec2.at(posInVec)(1,0)*setEigenVec2.at(posInVec)(1,0)));

				

				if(acos(cos_) < 0.0002){

					//ROS_INFO("cos: %f", acos(cos_)) ;

					//ROS_INFO("Point (%lf, %lf) is more likely to be the point (%lf, %lf). \nWith an angle of %lf between their normals!", (*p_set1)(0, i), (*p_set1)(1, i), (*p_set2)(0, posInVec), (*p_set2)(1, posInVec), acos(cos_));

					if(compSetCounter == 0){

						//cout << "test" << endl;

						if(comparisonSet != NULL) free(comparisonSet);

						//cout << "test2" << endl;

						comparisonSet = new MatrixXf(4, 1);

						//cout << "test3" << endl;

						compSetCounter++;

						//cout << "test4  compSetCounter : " << compSetCounter << endl;

					}

					else {

						//cout << "test5" << endl;

						comparisonSet->resize(4, ++compSetCounter);

						//cout << "test6" << endl;

					}

					//cout << "test7" << endl;
					(*comparisonSet)(0, compSetCounter-1) = (*p_set1)(0, i);
					//cout << "test8" << endl;
					(*comparisonSet)(1, compSetCounter-1) = (*p_set1)(1, i);
					//cout << "test9" << endl;
					(*comparisonSet)(2, compSetCounter-1) = (*p_set2)(0, posInVec);
					//cout << "test10" << endl;
					(*comparisonSet)(3, compSetCounter-1) = (*p_set2)(1, posInVec);


					//usedVecs.add(Vector2f((*p_set2)(0, posInVec), (*p_set2)(1, posInVec))); <<-- do it right!	
					//create the pair here!

				}
			
			}
		
		}
	
	}

	void publishMessage(Vector3f trans, const least_squares_rp_project::normals_ msg){

		least_squares_rp_project::pointAlignment new_msg;

		new_msg.header = msg.header;

		for(int i=0; i<p_set1->cols(); i++){
			
			new_msg.x_setSeqBase.push_back((*p_set1)(0,i));
			new_msg.y_setSeqBase.push_back((*p_set1)(1,i));

		}

		for(int j=0; j<p_set2->cols(); j++){

			new_msg.x_setSeqTrans.push_back((*p_set2)(0,j));
			new_msg.y_setSeqTrans.push_back((*p_set2)(1,j));

		}

		for(int k=0; k<3; k++){

			new_msg.transf.push_back(trans(k, 0));

		}

		pub.publish(new_msg);

	}

public:



	Aligner(const ros::Publisher pub){

		this->pub = pub;
		p_set1 = NULL;
		p_set2 = NULL;
		set1_free = false;
		comparisonSet = NULL;


	}


	void pointAlignerCB(const least_squares_rp_project::normals_ msg){

		cout << "Entering where I want to....\n\n" << endl;

		if(p_set1 == NULL){
		
			p_set1 = extractPointInfo(msg, &eingenValues1, &setEigenVec1);
		
			ofstream outPutFile;
		
			outPutFile.open("/home/spades/normals/mapCoords.dat",  std::ofstream::out);

			for(int i=0; i<p_set1->cols(); i++){

				outPutFile << (*p_set1)(0, i) << (*p_set1)(1, i) << setEigenVec1.at(i)(0, 0) << setEigenVec1.at(i)(1, 0) << "\n"; 

			}

			outPutFile.close();

		}
		else if(p_set2 == NULL){

			

			p_set2 = extractPointInfo(msg, &eingenValues2, &setEigenVec2);

			//cout << "Set 2: \n\n" << *p_set2 << endl;

			set1_free = true;

		}

		else if(set1_free){

			p_set1 = p_set2;
			setEigenVec1 = setEigenVec2;

			p_set2 = extractPointInfo(msg, &eingenValues2, &setEigenVec2);

		}

		if(p_set2 != NULL) {

			findRegionToCompare();
			Vector3f x; //initial guess

			x << 0,0,0;

			Vector4f tempResult;

			tempResult = Utils::pointAlignerLoop(x, *comparisonSet, comparisonSet->cols());

			//cout << "uhuuuuu" << endl;

			Vector3f result;



			for(int i=0; i<3; i++) 
				result(i,0) = tempResult(i,0);
			
			if(result(0,0) != result(0,0)) {

				result = x;


			}

			publishMessage(result, msg);

			cout << "result: \n" << result << endl;

			

		}

	}

};