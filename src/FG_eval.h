#ifndef FG_EVAL
#define FG_EVAL

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "constants.h"

using CppAD::AD;

/**
 * FG_eval class
 */
class FG_eval {

public:
  
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  
  // Constructor
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the MPC::Solve (function in the MPC.cpp file)
    
    // The cost is stored is the first element of `fg`, so the fg vector is 1
    // element larger than it was in MPC::Solve.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    
    // The part of the cost based on the reference state
    for (int t = 0; t < N; t++) {
      fg[0] += 2000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += 2000 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Minimize the use of actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 200 * CppAD::pow(
          vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    //--------------------
    // Setup Constraints
    //--------------------
    
    // Initial constraints
    
    // NOTE: We add 1 to each of the starting indices due to cost being located
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      
      //-------
      // State
      //-------
      
      // The state at time t + 1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // The state at time t
      AD<double> x0 = vars[x_start + (t - 1)];
      AD<double> y0 = vars[y_start + (t - 1)];
      AD<double> psi0 = vars[psi_start + (t - 1)];
      AD<double> v0 = vars[v_start + (t - 1)];
      AD<double> cte0 = vars[cte_start + (t - 1)];
      AD<double> epsi0 = vars[epsi_start + (t - 1)];
      
      //----------
      // Controls
      //----------
      
      // Controls at time t
      AD<double> delta0 = vars[delta_start + (t - 1)];
      AD<double> a0 = vars[a_start + (t - 1)];
      
      //-------------
      // Polynomial
      //-------------
      
      // Path from the polynomial at time t
      AD<double> f0 = coeffs[3] * x0 * x0 * x0 +
                      coeffs[2] * x0 * x0 +
                      coeffs[1] * x0 +
                      coeffs[0];
      
      // The atan of the first derivative of the polynomial at time t
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 +
                                       2 * coeffs[2] * x0 +
                                       coeffs[1]);
      
      
      // The idea here is to constraint this value to be 0.
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);
      fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + t] =
        cte1 - ( f0 - y0 + (v0 * CppAD::sin(epsi0)) * dt );
      fg[2 + epsi_start + t] =
        epsi1 - (y0 - psides0 + (v0 / Lf) * delta0 * dt);
    }
  }
};

#endif /* FG_EVAL */
