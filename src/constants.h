#ifndef ION_CONSTANTS_H_
#define ION_CONSTANTS_H_

namespace Cnst {

  //-----------------
  // Hyperparameters
  //-----------------

  // Prediction horizon: T = N * dt
  // It should be a few seconds at most. Beyond that horizon, the environment
  // will change enough that it won't make sense to predict any further into
  // the future
  
  // Number of steps in the prediction horizon
  // The goal of the MPC is to optimize controls. An optimizer will tune these
  // inputs until a low cost vector of control inputs is found. The length of
  // the vector is determined by N. Thus, N determines the number of variables
  // optimized by the MPC. This is also a major driver of computational cost
  size_t N = 10;
  
  // Timestep duration i.e. time elapsed between actuations
  // Larger values of dt result in less frequent actuations, which makes it
  // harder to accurately approximate a continuous reference trajectory. This
  // is sometimes called `discritization error`
  double dt = 0.1;
  
  // System latency from control signals to actuators response
  // In a real car, an actuation command won't execute instantly. There will
  // be a delay as the command propagates through the system. A realistic delay
  // might be on the order of 100 msec. MPC can adapt quite well because we can
  // model this latency into the system. One approach to solve this issue is to
  // run a simulation using the vehicle model starting from the current state
  // for the duration of the latency. The resulting state from the simulator
  // is the new initial state for MPC
  int latency = 0; // TODO: Set to 100 to test with latency and submit

  // This value assumes the model presented in the classroom is used. It was
  // obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on
  // a flat terrain. Lf was tuned until the the radius formed by the simulating
  // the model presented in the classroom matched the previous radius. This is
  // the length from front to CoG that has a similar radius
  const double Lf = 2.67;

  // References (i.e. our objectives)
  double ref_cte = 0.0;
  double ref_epsi = 0.0;
  double ref_v = 40.0; // TODO: This is in mph

  // Cost coefficients
  // The larger the coefficient of each cost, the lower the value we are willing
  // to accept on the respective error. Notice, that the optimum solution will
  // be the one with the lower cost value. Hence we heavily penalize the cost
  // functions with large coefficients but we are more reluctant to accept costs
  // with small coefficients
  int cte_param = 2000;
  int epsi_param = 2000;
  int v_param = 1;
  int delta_param = 5;
  int a_param = 5;
  int delta_seq_param = 200;
  int a_seq_param = 10;

  //-----------------
  // Array indices
  //-----------------

  // The solver takes all the state variables and actuator variables in a
  // singular vector. Thus, we should establish when one variable starts and
  // another ends to make our lifes easier
  size_t x_start = 0;
  size_t y_start = x_start + N;         // x are N
  size_t psi_start = y_start + N;       // y are N
  size_t v_start = psi_start + N;       // υ are N
  size_t cte_start = v_start + N;       // cte are N
  size_t epsi_start = cte_start + N;    // eψ are N
  size_t delta_start = epsi_start + N;  // δ are N-1
  size_t a_start = delta_start + N - 1; // α are N-1
  
  // 100ms latency equals the dt. Hence the fg index can be shifted from 1 to 2
  // to adjust for the latency as shown on the Q&A video. Possible values are:
  // 1 - no latency
  // 2 - latency of 100 msec
  size_t adjusted_t = 1;
}

#endif /* ION_CONSTANTS_H_ */
