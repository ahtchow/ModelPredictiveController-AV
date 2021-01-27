#ifndef MPC_HPP
#define MPC_HPP

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>

#include "Eigen-3.3/Eigen/Core"
#include "utils.h"

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
  double ref_vel; // Target Velocity of Vehicle used as reference

public:

  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  /**
   * @brief Construct a new MPC object
   * 
   */
  MPC(size_t N_, double dt_, double Lf_, double reference_velocity);

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
  double ref_vel; // Target Velocity of Vehicle used as reference

public: 

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  Eigen::VectorXd coeffs; // Fitted polynomial coefficients
  CppAD::AD<double> * cost = 0;
  double steering_tuning_factor = 1000.0; 
  
  /**
   * @brief Construct a new Evaluator object
   * @param coeffs - Fitted polynomial coefficients
   */
  Evaluator(Eigen::VectorXd coeffs_, 
            size_t N_, 
            double dt_, 
            double Lf_, 
            double reference_velocity);

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

  void operator()(ADvector & mpc_vector, const ADvector & state_actuator_vals){

    cost = &mpc_vector[0]; // First index is the cost of the MPC
    *cost = 0;

    /*****************************************************************************
     *                           COST CALCULATION                                *
     *****************************************************************************/

    for(size_t t = 0; t < N; t++){

      /* Cross Track Error */
      *cost += CppAD::pow(state_actuator_vals[cte_start+t], 2);
      /* Orientation Error */
      *cost += CppAD::pow(state_actuator_vals[epsi_start + t], 2);
      /* Deviation from Reference Speed */
      *cost += CppAD::pow(state_actuator_vals[v_start + t]-ref_vel, 2);    

    }

    // Minimize the use of actuators for stability.
    for (size_t t = 0; t < N -1; t++){

      *cost += CppAD::pow(state_actuator_vals[delta_start + t], 2);
      *cost += CppAD::pow(state_actuator_vals[a_start + t], 2);

    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {

      *cost += steering_tuning_factor * CppAD::pow(state_actuator_vals[delta_start + t + 1] - 
                                                              state_actuator_vals[delta_start + t], 2);
      *cost += CppAD::pow(state_actuator_vals[a_start + t + 1] - 
                                      state_actuator_vals[a_start + t], 2);

    }    

    /*****************************************************************************
     *                           SET-UP CONSTRAINTS                              *
     *****************************************************************************/

    /* Cost occupies first index, so offset by 1. Initialize Starting Values. */
    mpc_vector[1 + x_start] = state_actuator_vals[x_start];
    mpc_vector[1 + y_start] = state_actuator_vals[y_start];
    mpc_vector[1 + psi_start] = state_actuator_vals[psi_start];
    mpc_vector[1 + v_start] = state_actuator_vals[v_start];
    mpc_vector[1 + cte_start] = state_actuator_vals[cte_start];
    mpc_vector[1 + epsi_start] = state_actuator_vals[epsi_start];


    /* Computing the constraints for our horizon */
    for (size_t t = 1; t < N; ++t) {
 
      // The state at time t+1.
      CppAD::AD<double> x1 = state_actuator_vals[x_start + t];
      CppAD::AD<double> y1 = state_actuator_vals[y_start + t];
      CppAD::AD<double> psi1 = state_actuator_vals[psi_start + t];
      CppAD::AD<double> v1 = state_actuator_vals[v_start + t];
      CppAD::AD<double> cte1 = state_actuator_vals[cte_start + t];
      CppAD::AD<double> epsi1 = state_actuator_vals[epsi_start + t];

      // The state at time t.
      CppAD::AD<double> x0 = state_actuator_vals[x_start + t - 1];
      CppAD::AD<double> y0 = state_actuator_vals[y_start + t - 1];
      CppAD::AD<double> psi0 = state_actuator_vals[psi_start + t - 1];
      CppAD::AD<double> v0 = state_actuator_vals[v_start + t - 1];
      CppAD::AD<double> cte0 = state_actuator_vals[cte_start + t - 1];
      CppAD::AD<double> epsi0 = state_actuator_vals[epsi_start + t - 1];

      // Only consider the actuation at time t.
      CppAD::AD<double> delta0 = state_actuator_vals[delta_start + t - 1];
      CppAD::AD<double> a0 = state_actuator_vals[a_start + t - 1];

      CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      CppAD::AD<double> psides0 = CppAD::atan(coeffs[1]);

      /**
       * @brief Recall the equations for the model:
       * 
       * x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
       * y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
       * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
       * v_[t+1] = v[t] + a[t] * dt
       * cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
       * epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
       * 
       */

      /* Updating Constraints for timestep t+1 */
      mpc_vector[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      mpc_vector[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      mpc_vector[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      mpc_vector[1 + v_start + t] = v1 - (v0 + a0 * dt);
      mpc_vector[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      mpc_vector[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
      
    }

  };

};


#endif  // MPC_HPP


