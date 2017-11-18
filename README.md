# Model Predictive Control - MPC

## Introduction

This project implements a Model Predictive Controller to drive the car around the track. The cross track error is not provided by the simulator and is calculated from the code. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

The project rubric can be found [here](https://review.udacity.com/#!/rubrics/896/view)

 In this [youtube video](https://youtu.be/mcfGi2hPtuU) you can watch the car moving around the track under the MPC controller guidance. The yellow line depicts the desired trajectory and the green line is the MPC optimum trajectory.

[//]: # (Image References)
[image1]: ./mpc.png "MPC controller"
![alt text][image1]

## MPC Overview

The Model Predictive Controller calculates simulated actuator inputs, predicts the resulting trajectories and selects the trajectory with the minimum cost. In order to do so, It uses the desired trajectory from the path planning block and the current state of the vehicle as inputs. In a way, MPC reduces the control problem to an optimization problem of finding the best trajectory candidate.

In simple terms, imagine that we know the current state and the reference trajectory that we want to follow. We optimize our actuator inputs each step in time to minimize the cost of our predicted trajectory. Once we find the lowest cost trajectory we implement the respective actuators and throw away everything else.

At the heart of the MPC lie  the cost estimation functions. In essence these functions declare which errors are to be penalized the most in order to safely drive the vehicle. Our task is to allow for non-critical errors to occur while are keeping critical errors to a minimum.

## Hyperparameters

The MPC defines the prediction horizon *T* which is the duration over which future predictions are made. It should be a few seconds at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

![img](http://latex.codecogs.com/svg.latex?T%20%3D%20N%20*%20dt)

Where,

*N* is the number of steps in the prediction horizon. It determines the number of variables optimized by the MPC and is a major driver of computational cost.

*dt* is the timestep duration i.e. the time elapsed between actuations. Larger values of *dt* result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory.

During the implementation various values for *dt* were tested spanning from 0.8 to 1.4. However, 0.135 was found to help the model behave better than the other values from low speeds such as 40 mph to moderate speeds up to 100mph.

## Kinematic model

In order to implement the controller we first have to define a model of the vehicle dynamics and constraints. In this case we use a simple kinematic model that ignores tire forces, gravity and mass. While this simplification reduces the accuracy of the motion prediction, it makes it more tracktable. In addition, it is a good approximation of the actual vehicle dynamics in low and moderate speeds.

### State

The state vector contains the following variables: [*x, y, ψ, v, cte, eψ*], where:

*x* is the position of the car on the x-axis

*y* is  the position of the car on the y-axis

*ψ* is the orientation of the car

*v* is the speed of the car

*cte* is the ctoss-track error which expresses the lateral distance of the car from the center of the road

*eψ* is the orientation error which is the angular difference between the orientation of the car and the tangential angle of the route evaluated at the location of the car

### Controls or actuators

The control input vector contains the following variables: [*δ, α*], where:

*δ* is the steering angle

*α* is the acceleration modeling the throttle if positive or the brakes if negative

Note that If *δ* is positive we rotate counter-clockwise, or turn left. In the simulator however, a positive value implies a right turn and a negative value implies a left turn. This is incompatibility is corrected in the controller's implementation.

Since our model is nonholonomic i.e. the car cannot move in arbitrary directions, we have contraints both in steering angle and the acceleration:

![img](http://latex.codecogs.com/svg.latex?%5Cdelta%20%5Cin%20%5B-25%5Cdegree%2C%2025%5Cdegree%5D)

![img](http://latex.codecogs.com/svg.latex?%5Calpha%20%5Cin%20%5B-1%2C%201%5D)

### Vehicle physical characteristics

*Lf* measures the distance between the front of the vehicle and its Center of Gravity (CoG). The larger the vehicle, the slower the turn rate.

### Equations

Assuming that we have defined the state of the vehicle at time *t*, the following equations provide the state of the vehicle including the errors at the next time step, *t+1*:

![img](http://latex.codecogs.com/svg.latex?x_%7Bt%2B1%7D%20%3D%20x_t%20%2B%20v_t%20*%20cos(%5Cpsi_t)%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?y_%7Bt%2B1%7D%20%3D%20y_t%20%2B%20v_t%20*%20sin(%5Cpsi_t)%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?%5Cpsi_%7Bt%2B1%7D%20%3D%20%5Cpsi_t%20%2B%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?v_%7Bt%2B1%7D%20%3D%20v_t%20%2B%20%5Calpha%20*%20dt)

Error calculations at timestep t

![img](http://latex.codecogs.com/svg.latex?e%5Cpsi_t%20%3D%20%5Cpsi_%7Bt%7D%20-%20%20%5Cpsi%7Bdes%7D_t)

![img](http://latex.codecogs.com/svg.latex?%5Cpsi%7Bdes%7D_t%20%3D%20arctan(f%5E%7B%5Cprime%7D(x_t)))

![img](http://latex.codecogs.com/svg.latex?cte_t%20%3D%20f(x_t)%20-%20y_t)

Error calculations at timestep *t + 1*

![img](http://latex.codecogs.com/svg.latex?e%7B%5Cpsi%7D_%7Bt%2B1%7D%20%3D%20e%7B%5Cpsi%7D_t%20%2B%20%5Cfrac%7Bv_t%20%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?cte_%7Bt%2B1%7D%20%3D%20cte_t%20%2B%20v_t%20*%20sin(e%7B%5Cpsi%7D_t)%20*%20dt)

Expanded version with the error calculations with the timestep *t* substituted in the equations above

![img](http://latex.codecogs.com/svg.latex?e%7B%5Cpsi%7D_%7Bt%2B1%7D%20%3D%20%5Cpsi_t%20-%20arctan(f%5E%7B%5Cprime%7D(x_t))%20%2B%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![img](http://latex.codecogs.com/svg.latex?cte_%7Bt%2B1%7D%20%3D%20f(x_t)%20-%20y_t%20%2B%20v_t%20*%20sin(y_t%20-%20arctan(f%5E%7B%5Cprime%7D(x_t)))%20*%20dt)

## Latency

The latency of 100 msec is adapted into the system. For any given state, we update the car's position by running the motion equations for the duration of the latency. The state that is caculated through this procedure is then fed to the MPC controller as the present state. Because the *dt* hyperparameter of the MPC controller is in the order of 100msec, the latency is handled by shifting the timestep during the MPC update by 1.

## Polynomial of the 3rd order

The desired trajectory is passed as an input from the path planning block as a third order polynomial. In the simulator this line is depicted as yellow. We use this polynomial to extract the position of the car through the MPC implementation.

![img](http://latex.codecogs.com/svg.latex?f(x)%20%3D%20a_3%20*%20x%5E3%20%2B%20a_2%20*%20x%5E2%20%2B%20a_1%20*%20x%20%2B%20a_0)

![img](http://latex.codecogs.com/svg.latex?f%27(x)%20%3D%203%20*%20a_3%20*%20x%5E2%20%2B%202%20*%20a_2%20*%20x%20%2B%20a_1)

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
