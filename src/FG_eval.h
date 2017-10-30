#ifndef FG_EVAL
#define FG_EVAL

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

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
  }
};

#endif /* FG_EVAL */
