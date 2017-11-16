#ifndef MPC_CONSTANTS_H_
#define MPC_CONSTANTS_H_

namespace Cnst {
  
  const double mph_to_m_per_sec = 0.44704;
  
  //****************************************************************************
  // Hyperparameters
  //****************************************************************************

  // Prediction horizon: T = N * dt
  // It should be a few seconds at most. Beyond that horizon, the environment
  // will change enough that it won't make sense to predict any further into
  // the future
  
  // Number of steps in the prediction horizon
  // The goal of the MPC is to optimize controls. An optimizer will tune these
  // inputs until a low cost vector of control inputs is found. The length of
  // the vector is determined by N. Thus, N determines the number of variables
  // optimized by the MPC. This is also a major driver of computational cost
  const size_t N = 10;
  
  // Timestep duration i.e. time elapsed between actuations
  // Larger values of dt result in less frequent actuations, which makes it
  // harder to accurately approximate a continuous reference trajectory. This
  // is sometimes called `discritization error`
  const double dt = 0.135;
  // Values tested
  // 0.8  - tested - failed
  // 0.10 - tested - failed
  // 0.11 - tested - failed
  // 0.12 - tested - failed
  // 0.13 - tested - almost ok
  // 0.135- tested - ok
  // 0.14 - tested - ok
  
  // System latency from control signals to actuators response
  // In a real car, an actuation command won't execute instantly. There will
  // be a delay as the command propagates through the system. A realistic delay
  // might be on the order of 100 msec. MPC can adapt quite well because we can
  // model this latency into the system. One approach to solve this issue is to
  // run a simulation using the vehicle model starting from the current state
  // for the duration of the latency. The resulting state from the simulator
  // is the new initial state for MPC
  const double latency = 0.1; // In seconds
  const int latency_msec = 0.1 * 1000; // 100 msec as per the rubric

  // This value assumes the model presented in the classroom is used. It was
  // obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on
  // a flat terrain. Lf was tuned until the the radius formed by the simulating
  // the model presented in the classroom matched the previous radius. This is
  // the length from front to CoG that has a similar radius
  const double Lf = 2.67;

  // References (i.e. our objectives)
  const double ref_cte = 0.0;
  const double ref_epsi = 0.0;
  const double ref_v = 100.0 * mph_to_m_per_sec; // m/sec for the calculations
  // Values tested for speed from 40 to 100mph
  
  // Cost coefficients
  // The larger the coefficient of each cost, the lower the value we are willing
  // to accept on the respective error. Notice, that the optimum solution will
  // be the one with the lower cost value. Hence we heavily penalize the cost
  // functions with large coefficients but we are more reluctant to accept costs
  // with small coefficients
  const int cte_param = 3000;
  const int epsi_param = 4000;
  const int v_param = 1;
  const int delta_param = 5000;
  const int a_param = 1;
  const int delta_seq_param = 7000;
  const int a_seq_param = 1;
  // Values tested
  // cte   epsi  v    δ   α   dδ   dα
  // 3000, 3000, 1, 6000, 1, 5000, 1 -- 1st round ok but then crashes
  // 3000, 3000, 1, 2000, 1, 3000, 1 -- Crashes on first round
  // 3000, 3000, 1, 2000, 1, 5000, 1 -- Crashes on first round
  // 3000, 3000, 1, 7000, 1, 3000, 1 -- Crashes on first round
  // 3000, 3000, 1, 7000, 1, 6000, 1 -- 1st round ok but then crashes, better
  // 3000, 3000, 1, 5000, 1, 7000, 1 -- 1st round ok but then crashes, better
  
  //****************************************************************************
  // Array indices
  //****************************************************************************

  // The solver takes all the state variables and actuator variables in a
  // singular vector. Thus, we should establish when one variable starts and
  // another ends to make our lifes easier
  const size_t x_start = 0;
  const size_t y_start = x_start + N;         // x are N
  const size_t psi_start = y_start + N;       // y are N
  const size_t v_start = psi_start + N;       // υ are N
  const size_t cte_start = v_start + N;       // cte are N
  const size_t epsi_start = cte_start + N;    // eψ are N
  const size_t delta_start = epsi_start + N;  // δ are N-1
  const size_t a_start = delta_start + N - 1; // α are N-1
  
  // 100ms latency equals the dt. Hence the fg index can be shifted from 1 to 2
  // to adjust for the latency as shown on the Q&A video. Possible values are:
  // 1 - no latency
  // 2 - latency of 100 msec
  const size_t adjusted_t = 2;
}

#endif /* MPC_CONSTANTS_H_ */
