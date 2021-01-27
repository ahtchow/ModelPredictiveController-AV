#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include "Eigen-3.3/Eigen/Core"

using Eigen::VectorXd;


/**
 * @brief Evaluate polynomial given x-value
 * 
 * @param coeffs - coefficients to polynomial
 * @param x - input value
 * @return y values
 */
double Polyeval(const VectorXd &coeffs, double x){

  double result  = 0.0;
  for(int i = 0; i< coeffs.size(); ++i){
    result += coeffs[i] * pow(x,i);
  }
  
  return result;

}


/**
 * @brief Fit a polynomial with given parameters
 * 
 * @param xvals - x values
 * @param yvals - y values
 * @param order - order of polynomial
 * @return double - coefficients of polynomial
 */
VectorXd Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order){

  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;

}

#endif // POLYNOMIAL_H
