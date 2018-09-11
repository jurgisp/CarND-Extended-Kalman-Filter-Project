#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    return estimations.at(estimations.size() - 1);
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    return MatrixXd::Zero(4, 4);
}
