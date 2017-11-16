#ifndef FG_EVAL_
#define FG_EVAL_

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
  
  /**
   * Constructor
   */
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  /**
   * `fg` a vector of the cost constraints
   * `vars` is a vector of variable values (state & actuators)
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`, so the fg vector is 1
    // element larger than it was in MPC::Solve. Any additions to the cost are
    // be added to `fg[0]`
    fg[0] = 0;
    
    //**************************************************************************
    // Cost functions
    //**************************************************************************
    
    // The part of the cost based on the reference state
    for (int t = 0; t < Cnst::N; t++) {
      fg[0] +=  Cnst::cte_param *
                CppAD::pow(vars[Cnst::cte_start + t] - Cnst::ref_cte, 2);
      
      fg[0] +=  Cnst::epsi_param *
                CppAD::pow(vars[Cnst::epsi_start + t] - Cnst::ref_epsi, 2);
      
      fg[0] +=  Cnst::v_param *
                CppAD::pow(vars[Cnst::v_start + t] - Cnst::ref_v, 2);
    }
    
    // Minimize the use of actuators
    for (int t = 0; t < Cnst::N - 1; t++) {
      fg[0] +=  Cnst::delta_param *
                CppAD::pow(vars[Cnst::delta_start + t], 2);
      
      fg[0] +=  Cnst::a_param *
                CppAD::pow(vars[Cnst::a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations to penalize abrupt
    // movements and increase temporal smoothness
    for (int t = 0; t < Cnst::N - 2; t++) {
      fg[0] += Cnst::delta_seq_param *
               CppAD::pow(vars[Cnst::delta_start + t + 1] -
                          vars[Cnst::delta_start + t], 2);
      
      fg[0] += Cnst::a_seq_param *
               CppAD::pow(vars[Cnst::a_start + t + 1] -
                          vars[Cnst::a_start + t], 2);
    }
    
    ///**************************************************************************
    // Constraints
    //**************************************************************************
    
    // 0. Initial constraints
    // NOTE: We add 1 to each of the starting indices due to cost being located
    // at the begining of the fg array
    fg[1 + Cnst::x_start] = vars[Cnst::x_start];
    fg[1 + Cnst::y_start] = vars[Cnst::y_start];
    fg[1 + Cnst::psi_start] = vars[Cnst::psi_start];
    fg[1 + Cnst::v_start] = vars[Cnst::v_start];
    fg[1 + Cnst::cte_start] = vars[Cnst::cte_start];
    fg[1 + Cnst::epsi_start] = vars[Cnst::epsi_start];
    
    for (int t = 0; t < Cnst::N - 1; t++) {
      
      // 1. State constraints
      
      // The state at time t + 1
      AD<double> x1 = vars[Cnst::x_start + (t + 1)];
      AD<double> y1 = vars[Cnst::y_start + (t + 1)];
      AD<double> psi1 = vars[Cnst::psi_start + (t + 1)];
      AD<double> v1 = vars[Cnst::v_start + (t + 1)];
      AD<double> cte1 = vars[Cnst::cte_start + (t + 1)];
      AD<double> epsi1 = vars[Cnst::epsi_start + (t + 1)];
      
      // 2. The state at time t
      AD<double> x0 = vars[Cnst::x_start + t];
      AD<double> y0 = vars[Cnst::y_start + t];
      AD<double> psi0 = vars[Cnst::psi_start + t];
      AD<double> v0 = vars[Cnst::v_start + t];
      AD<double> cte0 = vars[Cnst::cte_start + t];
      AD<double> epsi0 = vars[Cnst::epsi_start + t];
      
      // 3. Controls constraints
      
      // Controls at time t
      AD<double> delta0 = vars[Cnst::delta_start + t];
      AD<double> a0 = vars[Cnst::a_start + t];
      
      // 4. Mathematical model constraints
      
      // Path from the polynomial at time t
      // f(x) = ax^3 + bx^2 + cx + d
      AD<double> f0 = coeffs[3] * x0 * x0 * x0 +
                      coeffs[2] * x0 * x0 +
                      coeffs[1] * x0 +
                      coeffs[0];
      
      // The atan of the first derivative of the polynomial at time t
      // f'(x) = 3ax^2 + 2bx +c
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 +
                                       2 * coeffs[2] * x0 +
                                       coeffs[1]);
      
      // The idea here is to constraint this value to be 0.
      // NOTE: The use of `AD<double>` and use of `CppAD`! This is also CppAD
      // can compute derivatives and pass these to the solver
      fg[Cnst::adjusted_t + Cnst::x_start + t] =
        x1 - (x0 + v0 * CppAD::cos(psi0) * Cnst::dt); // = 0
      
      fg[Cnst::adjusted_t + Cnst::y_start + t] =
        y1 - (y0 + v0 * CppAD::sin(psi0) * Cnst::dt); // = 0
      
      fg[Cnst::adjusted_t + Cnst::psi_start + t] =
        psi1 - (psi0 + v0 * (-delta0) * Cnst::dt / Cnst::Lf); // = 0
      
      fg[Cnst::adjusted_t + Cnst::v_start + t] =
        v1 - (v0 + a0 * Cnst::dt); // = 0
      
      fg[Cnst::adjusted_t + Cnst::cte_start + t] =
        cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0)) * Cnst::dt); // = 0
      
      fg[Cnst::adjusted_t + Cnst::epsi_start + t] =
        epsi1 - (psi0 - psides0 + v0 * (-delta0) * Cnst::dt / Cnst::Lf); // = 0
    }
  }
};

#endif /* FG_EVAL_ */
