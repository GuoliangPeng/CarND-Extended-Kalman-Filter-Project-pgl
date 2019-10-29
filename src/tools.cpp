#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd::Zero(3, 4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//TODO: YOUR CODE HERE 
	double pxpy = px*px+py*py;
	double sqrtpxpy = sqrt(pxpy);
	double threebytwo = pxpy*sqrtpxpy;

	//check division by zero
	if(fabs(pxpy)<0.0001){
	    cout<<"CalculateJacobian() - Error - Division by zero";
	} else {
	    Hj<< px/sqrtpxpy, py/sqrtpxpy, 0, 0, 
			-py/pxpy, px/pxpy, 0, 0,
			py* (vx*py-px*vy) / threebytwo, px* (vy*px-py*vx) / threebytwo, px/sqrtpxpy, py/sqrtpxpy;
	}
  return Hj;
}
