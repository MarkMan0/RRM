#ifndef TRANSFORM_MATRIX_H
#define TRANSFORM_MATRIX_H

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace matrix
{
Eigen::MatrixXd createRz(double angle);
Eigen::MatrixXd createRy(double angle);
Eigen::MatrixXd createTz(double l);
}  // namespace matrix

#endif  // TRANSFORM_MATRIX_H
