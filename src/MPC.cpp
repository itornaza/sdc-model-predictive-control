#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "FG_eval.h"
#include "constants.h"

using CppAD::AD;
using namespace std;

/**
 * MPC class implementation
 */

// Constructor
MPC::MPC() {}

// Destructor
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Get the state initial values from the state vector
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  // Set the number of model variables (includes both states and inputs)
  size_t n_vars = Ct::N * 6 + (Ct::N - 1) * 2;
  
  // Set the number of constraints
  size_t n_constraints = Ct::N * 6;
  
  // Initial value of the independent variables. SHOULD BE 0 except init state
  Dvector vars(n_vars);
  for (int ix = 0; ix < n_vars; ix++) {
    vars[ix] = 0;
  }

  // Initial state values
  vars[Ct::x_start] = x;
  vars[Ct::y_start] = y;
  vars[Ct::psi_start] = y;
  vars[Ct::v_start] = v;
  vars[Ct::cte_start] = cte;
  vars[Ct::epsi_start] = epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int ix = 0; ix < Ct::delta_start; ix++) {
    vars_lowerbound[ix] = -std::numeric_limits<double>::max();
    vars_upperbound[ix] = std::numeric_limits<double>::max();
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int ix = Ct::delta_start; ix < Ct::a_start; ix++) {
    vars_lowerbound[ix] = -0.436332;
    vars_upperbound[ix] = 0.436332;
  }
  
  // Acceleration/decceleration upper and lower limits.
  for (int ix = Ct::a_start; ix < n_vars; ix++) {
    vars_lowerbound[ix] = -1.0;
    vars_upperbound[ix] = 1.0;
  }
  
  // Lower and upper limits for constraints
  // All of these should be 0 except the initial state indices that are set
  // just after
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int ix = 0; ix < n_constraints; ix++) {
    constraints_lowerbound[ix] = 0;
    constraints_upperbound[ix] = 0;
  }
  
  // Set the initial state upper bounds so the solver knows where to start from
  constraints_lowerbound[Ct::x_start] = x;
  constraints_lowerbound[Ct::y_start] = y;
  constraints_lowerbound[Ct::psi_start] = psi;
  constraints_lowerbound[Ct::v_start] = v;
  constraints_lowerbound[Ct::cte_start] = cte;
  constraints_lowerbound[Ct::epsi_start] = epsi;
  
  // Set the initial state lower bounds
  constraints_upperbound[Ct::x_start] = x;
  constraints_upperbound[Ct::y_start] = y;
  constraints_upperbound[Ct::psi_start] = psi;
  constraints_upperbound[Ct::v_start] = v;
  constraints_upperbound[Ct::cte_start] = cte;
  constraints_upperbound[Ct::epsi_start] = epsi;
  
  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // Options for IPOPT solver
  string options;
  options += "Integer print_level  0\n";
  
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  
  // The solver has a maximum time limit of 0.5 seconds
  options += "Numeric max_cpu_time          0.5\n";

  // Place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval::FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  cout << "Cost " << cost << endl;
  
  // Return the vector containing the controls and points to display
  // result = { δ, α, x_1, y_1, x_2, y_2, ..., x_n, y_n }
  vector<double> result;
  
  // Controls: { δ, α }
  result.push_back(solution.x[Ct::delta_start]);
  result.push_back(solution.x[Ct::a_start]);
  
  // Points: { x_1, y_1, x_2, y_2, ..., x_n, y_n }
  for (int t = 0; t < Ct::N; ++t) {
    result.push_back(solution.x[Ct::x_start + t]);
    result.push_back(solution.x[Ct::y_start + t]);
  }
  
  return result;
}
