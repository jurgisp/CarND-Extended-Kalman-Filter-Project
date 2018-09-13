#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    auto n = estimations.size();
    VectorXd sum = VectorXd::Zero(4);
    for (size_t i = 0; i < n; i++) {
        VectorXd diff = estimations.at(i) - ground_truth.at(i);
        sum = sum + diff.cwiseProduct(diff);
    }

    sum = sum / n;
    sum = sum.cwiseSqrt();
    return sum;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x) {
    double px = x(0), py = x(1), vx = x(2), vy = x(3);
    double r2 = pow(px, 2) + pow(py, 2);
    double r = sqrt(r2);
    double r3 = r2*r;

    MatrixXd Hj(3, 4);
    Hj <<  px/r,   py/r, 0, 0,
          -py/r2, px/r2, 0, 0,
          py*(vx*py-vy*px)/r3, px*(vy*px-vx*py)/r3, px/r, py/r;
    return Hj;
}
