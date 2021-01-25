#include "utils.h"

constexpr double pi() { 
    return M_PI; 
}


double deg2rad(double x) {
    return x * pi() / 180; 
}


double rad2deg(double x) { 
    return x * 180 / pi(); 
}


string hasData(string s){
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.

    auto found_null = s.find("null"); // Search for null in string
    auto b1 = s.find_first_of("["); // Search for start
    auto b2 = s.rfind("}]"); // Search for ending

    if(found_null != string::npos){
        // Found null, meaning invalid string data
        return "";
    } else if ( b1 != string::npos && b2 != string::npos) {
        // string substr (size_t pos = 0, size_t len = npos) const;
        // return string starting from b1 and ending at b2.
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";

}


double Polyeval(const Eigen::VectorXd &coeffs, double x){

  double result  = 0.0;
  for(int i = 0; i< coeffs.size(); ++i){
    result += coeffs[i] * pow(x,i);
  }
  
  return result;

}


Eigen::VectorXd Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order){

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