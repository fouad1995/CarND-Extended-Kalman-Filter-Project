#include "kalman_filter.h"

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
    

    // map the state vector to measurment vector 
    VectorXd z_predicted = H_ * x_; 

    // calculate erro in prediction  (actual sensor reading - predicted value from predicted state )
    VectorXd y = z - z_predicted; 

    // calculate S matrix
    // adding R_ which has information about Measurment noise
    MatrixXd S = H_ * P_ * H_.transpose() + R_;

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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
