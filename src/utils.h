#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include "Eigen-3.3/Eigen/Core"

using std::string;

constexpr double pi();

/**
 * @brief Convert from Degrees to Radians
 * 
 * @param x 
 * @return double 
 */
double deg2rad(double x);

/**
 * @brief Convert from Radians to Degrees
 * 
 * @param x 
 * @return double 
 */
double rad2deg(double x);


/**
 * @brief Checks if the SocketIO event has JSON data.
 *          
 * @param s 
 * @return string 
 */
string hasData(string s);

/**
 * @brief Evaluate polynomial given x-value
 * 
 * @param coeffs - coefficients to polynomial
 * @param x - input value
 * @return y values
 */
double Polyeval(const Eigen::VectorXd &coeffs, double x);

/**
 * @brief Fit a polynomial with given parameters
 * 
 * @param xvals - x values
 * @param yvals - y values
 * @param order - order of polynomial
 * @return double - coefficients of polynomial
 */
Eigen::VectorXd Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);


#endif // UTILS_HPP
