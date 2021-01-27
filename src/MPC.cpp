#include "MPC.h"


MPC::MPC(size_t N_, double dt_, double Lf_, double reference_velocity) {
    
  N = N_;
  dt = dt_;
  Lf = Lf_;
  ref_vel = reference_velocity;

  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;

}


MPC::~MPC() {}


std::vector<double> MPC::Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs){

  size_t n_states = N * 6; // Number of States
  size_t n_actuators = (N - 1) * 2; // Numver of Actuators
  size_t n_variables = n_states + n_actuators; // Number of Variables
  size_t n_constraints = n_states; // Number of Constraints

  /* Initial Values */
  double x = state[0]; // X-Position
  double y = state[1]; // Y-Position
  double psi = state[2]; // Heading
  double v = state[3]; // Velocity
  double cte = state[4]; // Cross Track Error
  double epsi = state[5]; // Orientation Error

  /* Initialize value of the independent variables. */
  Dvector mpc_states(n_variables);
  for (size_t i = 0; i < n_variables; ++i) {
    mpc_states[i] = 0.0;
  }

  /* Fill in values for initial state */
  mpc_states[x_start] = x;
  mpc_states[y_start] = y;
  mpc_states[psi_start] = psi;
  mpc_states[v_start] = v;
  mpc_states[cte_start] = cte;
  mpc_states[epsi_start] = epsi;

  /* Lower and upper limits for all state and actuator values in environment*/
  Dvector env_lowerbound(n_variables);
  Dvector env_upperbound(n_variables);

  /* Set all non-actuator values to max */
  for (size_t i = 0; i < delta_start; ++i) {
    env_lowerbound[i] = -INT_MAX;
    env_upperbound[i] = INT_MAX;
  }

  /* Set steering actuator limtis between [-25, 25] degrees */
  for (size_t i = delta_start; i < a_start; ++i) {
    env_lowerbound[i] = -deg2rad(25);
    env_upperbound[i] = deg2rad(25);
  } 

  /* Set acceleration actuator limtis between [-1, 1] */
  for (size_t i = a_start; i < n_variables; ++i) {
    env_lowerbound[i] = -1.0;
    env_upperbound[i] = 1.0;
  }

  /* State Variable Constraints */
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  /* These define the state limits for the next N points, initialize to 0. */
  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  /* Set Initial Lowerbound */
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  /* Set Initial Upperbound */
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  /* Optimizer to compute objectives and constraints */
  Evaluator mpc_evaluator(coeffs, N, dt, Lf, ref_vel);
  

  /**
   * @brief Define options for optimizer
   * 
   * Setting sparse to true allows the solver to take advantage
   * of sparse routines, this makes the computation MUCH FASTER.
   * 
   */
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  /* Place to return solution */
  CppAD::ipopt::solve_result<Dvector> solution;

  /* Solve the problem optimizion problem */
  CppAD::ipopt::solve<Dvector, Evaluator>(
      options, mpc_states, env_lowerbound, env_upperbound, constraints_lowerbound,
      constraints_upperbound, mpc_evaluator, solution);


  /* Check some of the solution values */
  bool steady_state = true;
  steady_state &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /* Return Actuator estimates at next time step */
  return std::vector<double>{solution.x[delta_start], solution.x[a_start]};
}


Evaluator::Evaluator(Eigen::VectorXd coeffs_, 
                     size_t N_,
                     double dt_,
                     double Lf_,
                     double reference_velocity){
  
  N = N_;
  dt = dt_;
  Lf = Lf_;
  ref_vel = reference_velocity;

  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;
  coeffs = coeffs_;

}
  
Evaluator::~Evaluator() {

  if(cost) free(cost);
  cost = NULL;

}


