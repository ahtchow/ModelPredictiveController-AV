#ifndef MPC_HPP
#define MPC_HPP

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>

#include "Eigen-3.3/Eigen/Core"

typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {

private:

  size_t N; // Duration
  double dt; // Timestep
  double Lf; // This is the length from front to CoG. (Similar to Radius)
  size_t x_start; // Reference for start of x-values in states vector
  size_t y_start; // Reference for start of y-values in states vector
  size_t psi_start; // Reference for start of heading values in states vector
  size_t v_start; // Reference for start of velocity values in states vector
  size_t cte_start; // Reference for start of cross track error values in states vector
  size_t epsi_start; // Reference for start of orientation error values in states vector
  size_t delta_start; // Reference for start of heading actuators values in states vector
  size_t a_start; // Reference for start of acceleration actuators values in states vector

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


