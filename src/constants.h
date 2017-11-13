#ifndef ION_CONSTANTS_H_
#define ION_CONSTANTS_H_

/**
 * Hyperparameters
 */
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used. It was
// obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain. Lf was tuned until the the radius formed by the simulating the
// model presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// References (i.e. our objectives)
double ref_cte = 0.0;
double ref_epsi = 0.0;
double ref_v = 40.0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// when one variable starts and another ends to make our lifes easier
size_t x_start = 0;
size_t y_start = x_start + N;         // x are N
size_t psi_start = y_start + N;       // y are N
size_t v_start = psi_start + N;       // υ are N
size_t cte_start = v_start + N;       // cte are N
size_t epsi_start = cte_start + N;    // eψ are N
size_t delta_start = epsi_start + N;  // δ are N-1
size_t a_start = delta_start + N - 1; // α are N-1

#endif /* ION_CONSTANTS_H_ */
