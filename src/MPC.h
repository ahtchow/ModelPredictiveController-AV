#ifndef MPC_HPP
#define MPC_HPP

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>

#include "Eigen-3.3/Eigen/Core"

typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

class MPC {

private:

  const size_t N; // Duration
  const double dt; // Timestep
  const double Lf; // This is the length from front to CoG. (Similar to Radius)

public:
  
  /**
   * @brief Construct a new MPC object
   * 
   */
  MPC(size_t N_, double dt_, double Lf_);

  /**
   * @brief Destroy the MPC object
   * 
   */
  virtual ~MPC();

  /**
   * @brief Solve the model given an initial state and polynomial coefficients.
   * 
   * @param state - State [x,y,ψ,v,cte,eψ]
   * @param coeffs - Coefficients of fitted polynomial
   * @return Return the first actuations.
   */
  std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);

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

};

class Evaluator {

public: 

  Eigen::VectorXd coeffs; // Fitted polynomial coefficients
  double * cost;
  
  /**
   * @brief Construct a new Evaluator object
   * @param coeffs - Fitted polynomial coefficients
   */
  Evaluator(Eigen::VectorXd coeffs_);

  /**
   * @brief Destroy the Evaluator object
   * 
   */
  virtual ~Evaluator();

  /**
   * @brief Update Cost, Set-up Constraints,
   * 
   * @param mpc_vector - vector containing the cost and constraints.
   * @param state_actuator_vals - containing the variable values (state & actuators).
   */

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector & mpc_vector, const ADvector & state_actuator_vals);

};


#endif  // MPC_HPP


