#include "MPC.h"

MPC::MPC(size_t N_, double dt_, double Lf_) {
    
  N = N_;
  dt = dt_;
  Lf = Lf_;

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

  bool steady_state = true; // Status
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
  for (int i = 0; i < n_variables; ++i) {
    mpc_states[i] = 0.0;
  }

  /* Fill in values for initial state */
  mpc_states[x_start] = x;
  mpc_states[y_start] = y;
  mpc_states[psi_start] = psi;
  mpc_states[v_start] = v;
  mpc_states[cte_start] = cte;
  mpc_states[epsi_start] = epsi;

  /* Lower and upper limits for all state values */
  Dvector vars_lowerbound(n_variables);
  Dvector vars_upperbound(n_variables);

    

  return std::vector<double>(0.0);

}


