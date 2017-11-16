# Model Predictive Control

## Introduction

This project implements Model Predictive Control to drive the car around the track. The cross track error is not provided by the simulator and is calculated from the code. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

The project rubric can be found [here](https://review.udacity.com/#!/rubrics/896/view)

## Equations that describe the model

### Kinematic model

![img](http://latex.codecogs.com/svg.latex?x_%7Bt%2B1%7D%20%3D%20x_t%20%2B%20v_t%20*%20cos(%5Cpsi_t)%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?y_%7Bt%2B1%7D%20%3D%20y_t%20%2B%20v_t%20*%20sin(%5Cpsi_t)%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?%5Cpsi_%7Bt%2B1%7D%20%3D%20%5Cpsi_t%20%2B%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?v_%7Bt%2B1%7D%20%3D%20v_t%20%2B%20%5Calpha%20*%20dt)

### Polynomial of the 3rd order

![img](http://latex.codecogs.com/svg.latex?f(x)%20%3D%20a_3%20*%20x%5E3%20%2B%20a_2%20*%20x%5E2%20%2B%20a_1%20*%20x%20%2B%20a_0)

![img](http://latex.codecogs.com/svg.latex?f%27(x)%20%3D%203%20*%20a_3%20*%20x%5E2%20%2B%202%20*%20a_2%20*%20x%20%2B%20a_1)

### Error calculations at timestep t

![img](http://latex.codecogs.com/svg.latex?e%5Cpsi_t%20%3D%20%5Cpsi_%7Bt%7D%20-%20%20%5Cpsi%7Bdes%7D_t)

![img](http://latex.codecogs.com/svg.latex?%5Cpsi%7Bdes%7D_t%20%3D%20arctan(f%5E%7B%5Cprime%7D(x_t)))

![img](http://latex.codecogs.com/svg.latex?cte_t%20%3D%20f(x_t)%20-%20y_t)

### Error calculations at timestep (t + 1)

Depending on the previous step

![img](http://latex.codecogs.com/svg.latex?e%7B%5Cpsi%7D_%7Bt%2B1%7D%20%3D%20e%7B%5Cpsi%7D_t%20%2B%20%5Cfrac%7Bv_t%20%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?cte_%7Bt%2B1%7D%20%3D%20cte_t%20%2B%20v_t%20*%20sin(e%7B%5Cpsi%7D_t)%20*%20dt)

Expanded version with the previous step substituted in the equations above

e{\psi}_{t+1} = \psi_t - arctan(f^{\prime}(x_t)) + \frac{v_t}{L_f} * \delta_t * dt

![img](http://latex.codecogs.com/svg.latex?e%7B%5Cpsi%7D_%7Bt%2B1%7D%20%3D%20%5Cpsi_t%20-%20arctan(f%5E%7B%5Cprime%7D(x_t))%20%2B%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

cte_t+1 = f(x_t) - y_t + υ_t * sin(y_t - arctan(f^{\prime}(x_t))) * dt

![img](http://latex.codecogs.com/svg.latex?cte_t%2B1%20%3D%20f(x_t)%20-%20y_t%20%2B%20%CF%85_t%20*%20sin(y_t%20-%20arctan(f%5E%7B%5Cprime%7D(x_t)))%20*%20dt)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps (not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

