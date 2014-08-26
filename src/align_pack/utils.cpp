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

#include <time.h>

#include "least_squares_rp_project/normals_.h"
#include "least_squares_rp_project/pointAlignment.h"

#define PI 3.14159265

using namespace Eigen;
using namespace std;

#ifndef UTILS_H
#define UTILS_H
#endif

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > listVec2f;
typedef Matrix<float, 2, 3> Jacobian;

namespace Utils{

	Matrix3f v2t(Vector3f vec){

		Matrix3f tempMatrix;

		tempMatrix << cos(vec(2,0)*PI/180), (-sin(vec(2,0)*PI/180)), vec(0,0),
				   	  sin(vec(2,0)*PI/180), cos(vec(2,0)*PI/180) , vec(1,0),
				   	  0	, 0, 1; 


		return tempMatrix;

	}

	Matrix3f v2tRad(Vector3f vec){

		//cout << "from inside the v2tRad: \n" << vec << endl;

		Matrix3f tempMatrix;

		tempMatrix << cos(vec(2,0)), (-sin(vec(2,0))), vec(0,0),
				   	  sin(vec(2,0)),   cos(vec(2,0)) , vec(1,0),
				   	  0	, 0, 1; 


		return tempMatrix;

	}

	Vector3f t2v(Matrix3f mat){

		Vector3f tempVec;

		tempVec(0, 0) = mat(0, 2);
		tempVec(1, 0) = mat(1, 2);
		tempVec(2, 0) = atan2(mat(1, 0), mat(0, 0));		

		return tempVec;

	}

	Vector2f computeError(int i, Matrix3f X, MatrixXf Z){

		Vector3f pi(1, 1, 1);
		Vector3f pj(1, 1, 1);

		for(int j=0; j<2; j++){

			pi(j, 0) = Z(j, i);
			pj(j, 0) = Z(j+2, i); 

		}

		Vector3f efull = X*pi - pj;
		Vector2f e;

		e(0,0) = efull(0,0);
		e(1,0) = efull(1,0);

		return e;

	}

	Jacobian computeJacobian(int i, Matrix3f X, MatrixXf Z){

		Jacobian jac;

		float c = X(0,0);
		float s = X(1,0);

		Matrix2f Rprime;

		jac << 1, 0, 0, 0, 1, 0;

		Rprime << -s, -c, c, -s;

		Vector2f tempVec(Z(0, i), Z(1, i));

		tempVec = Rprime*tempVec;

		jac(0, 2) = tempVec(0,0);
		jac(1, 2) = tempVec(1,0);

		return jac;

	}

	Vector4f pointAlignerIteration(Vector3f x, MatrixXf Z){

		Matrix3f H;
		Vector3f b;
		Vector4f xNew_chi;

		H << 0,0,0,
			 0,0,0,
			 0,0,0;

		b << 0,
			 0,
			 0;


		float chi = 0;

		Matrix3f X = v2tRad(x);

		//cout << "matrix X from inside the point align iteration: \n" << X << endl;

		Matrix2f Omega;

		Omega << 1, 0,
				 0, 1;

		Vector2f e;
		Jacobian jac;

		for(int i=0; i < Z.cols(); i++){

			e = computeError(i, X, Z);
			jac = computeJacobian(i, X, Z);

			//cout << "jacobian: \n" << jac << endl;

			//getc(stdin);

			H += jac.transpose()*Omega*jac;
			b += jac.transpose()*Omega*e;

			chi += e.transpose()*Omega*e;

		}

		Vector3f dX;

		dX = -H.inverse()*b;

		//cout << "H inverse: \n" << H.inverse() << endl;	

		//getc(stdin);

		for(int i=0; i<3; i++){

			xNew_chi(i, 0) = x(i, 0) + dX(i, 0);

		} 

		xNew_chi(2, 0) = atan2(sin(xNew_chi(2,0)), cos(xNew_chi(2, 0)));

		xNew_chi(3, 0) = chi;

		return xNew_chi;

	}

	Vector4f pointAlignerLoop(Vector3f x, MatrixXf Z, int iterations){

		Vector3f xNew = x;
		Vector4f xNew_chi;

		for(int i=0; i<iterations; i++){

			xNew_chi = pointAlignerIteration(xNew, Z);

			for(int j=0; j<3; j++) xNew(j, 0) = xNew_chi(j,0);

		}

		return xNew_chi;

	}


	MatrixXf transformPoints(Matrix3f X, MatrixXf P){

		MatrixXf Pfull(3, P.cols());

		for(int i=0; i<P.cols(); i++){

			Pfull(0, i) = P(0, i);
			Pfull(1, i) = P(1, i);
			Pfull(2, i) = 1;
		}

		Pfull = X*Pfull;

		MatrixXf Pt(2, P.cols());

		for(int i=0; i<P.cols(); i++){

			Pt(0, i) = Pfull(0, i);
			Pt(1, i) = Pfull(1, i);

		}

		return Pt;

	}

	void PointAlignerTest(){

		MatrixXf P(2, 100);

		for(int i=0; i<100; i++){
		
			srand (time(NULL));
			P(0, i) = rand()%100;

			for(int j=0; j<100000; j++)

			srand (time(NULL));
			P(1, i) = rand()%100;
		
		}

		cout << "P : \n" << P << endl;

		Vector3f x_ideal;

		x_ideal << 20, 30, PI/2;

		//cout << "x_ideal : \n" << x_ideal << endl;

		Matrix3f X_ideal;

		X_ideal = v2tRad(x_ideal);

		//cout << "X_ideal : \n" << X_ideal << endl;

		MatrixXf Pt(2, 100);

		Pt = transformPoints(X_ideal.inverse(), P);

		cout << "Pt : \n" << Pt << endl;

		MatrixXf Z(4, P.cols());

		for(int i=0; i<P.cols(); i++){

			Z(0, i) = P(0, i);
			Z(1, i) = P(1, i);
			Z(2, i) = Pt(0, i);
			Z(3, i) = Pt(1, i);

		}

		cout << "Z : \n" << Z << endl;

		Vector3f x; //initial guess

		x << 0,0,0;

		cout << "x : \n" << x << endl;

		cout << pointAlignerLoop(x, Z, 100);

		return ;
	}


}