#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0); // px ( position in x )
    float py = x_state(1); // py ( position in y )
    float vx = x_state(2); // vx ( velocity in x )
    float vy = x_state(3); // vy ( velocity in y )

    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    // check division by zero
    if (fabs(c1) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
         -(py / c1), (px / c1), 0, 0,
          py* (vx * py - vy * px) / c3, px* (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;

}


double Tools::normalize(double value, double min, double max) {

    return (value - min) / (max - min);

}