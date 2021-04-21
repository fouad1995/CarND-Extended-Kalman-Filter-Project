#include "kalman_filter.h"
#include "tools.h" // to calculate jacobian
#include <iostream>
#include <algorithm>    // std::max
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
    // predict the State ( using linear model to calculate the position and velocity ) 

    x_ = F_ * x_;  

    // update covariance matrix to track the uncertainity 
    // adding Q_ which has information about process noise
    P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    // calculate erro in prediction  (actual sensor reading - predicted value from predicted state (using JACOBIAN )
    // H_ will be passed from outside in initialization so don't worry this will be correct value here H_(3*4)

    VectorXd y = z - H_*x_;
  
  	UpdateKalman(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    // map the state vector to measurment vector ( convert from polar coordinate to cartesian coordinate )

    float px = x_(0);    // position in x direction
    float py = x_(1);    // position in y direction
    float vx = x_(2);    // velocity in x direction 
    float vy = x_(3);    // velocity in y direction 

    
    float magnitude = sqrt((px * px) + (py * py));

    float rho = magnitude;                                   // sqrt(px^2+py^2)
    float phi = atan2(py,px);                                // arctan(py/px) => radians 
  
    // check devision  by zero 
    // leave the equation to take the decision on what to do with zaro
    float rho_dot = (px * vx + py * vy) / std::max(rho,(float)0.0001);

    VectorXd h_x = VectorXd(3);   // holds the mapped data from polar to cartesian coordinates
	
    h_x << rho,
           phi,
           rho_dot;

   // calculate erro in prediction  (actual sensor reading (mapped from polar to cartesian coordinates) - predicted value from predicted state )
    VectorXd y = z - h_x;

  // normalizing angles 
  while(y(1) > M_PI || y(1) < -M_PI)  y(1) = tools.normalize(y(1),-M_PI,M_PI);
  
  UpdateKalman(y);

}


void KalmanFilter::UpdateKalman(const VectorXd& y){
  
    // calculate S matrix
    // adding R_ which has information about Measurment noise

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;

  
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;

    // calculate kalman gain
    // Kalman Gain should have information about P and R to decide which will be trusted most (measurment or prediction)
    MatrixXd K = PHt * Si;

    // update new state
    // KG will give a very heigh weight to measurment if Q is low , P is high and vice verse
    x_ = x_ + (K * y);

    // update the Covariance matrix 
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;   
  
}
