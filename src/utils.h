#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <math.h>

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


#endif // UTILS_H
