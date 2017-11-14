#ifndef MPC_H_
#define MPC_H_

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {

public:
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients
  // Return the first actuatotions
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H_ */
