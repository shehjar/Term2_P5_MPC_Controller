# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

### The Model

The model hence developed is based on the kinematic model setup as viewed from the lecture. This includes the equations of motion that captures the general movement of the car, given its coordinates, position, velocity and orientation. Therefore the state vector of the car was chosen to be 

* `px` as the position of the car in x direction.
* `py` as the position of the car in y direction.
* `psi` as the orientation angle of the car with respect to the horizontal axis in x direction.
* `v` as the velocity of the car.
* `cte` as the cross track error of the car with respect to the planned trajectory.
* `epsi` as the error in the orientation of the car.

The actuators were therfore chosen to be

* `steering_value` or `delta`  as the steering angle of the car
* `thottle_value` as the accelaration input to the car in the simulator.

The update equations were also deployed as given in the lectures as-
```
px += v * cos(psi) * dt
py += v * sin(psi) * dt
psi += v * steer_value / Lf * dt
v += throttle_value * dt
```

The cross track error is updated by calculating the distance between the predicted position of the car with respect to the planned trajectory. This is calculated using a polynomial fitting using the predicted position `px`. Similarly to this the error in the orientation `epsi` is updated using the same equation as `psi`, with the only difference being that it takes into account the desired `psi` orientation. This is evaluated by the substituting the the position `px` within the tangent of the fitted polynomial.

### Timestep Length and Elapsed Duration (N & dt)

The model predictive control works in a way to transform a control problem to an optimization problem. The way this is achieved through discretizing the predicted movement of the car in `N` steps. A time step `dt` is thus introduced to update the kinematic equations that describe the motion of the car. For the parameter tuning the parameters `N` and `dt` were tested from the range of 5 to 10 and 0.1 to 0.2 respectively. Finally the values of 10 and 0.1 were selected.

### Polynomial Fitting and MPC Pre-processing

The planned trajectory contains data that can be used to fit a polynomial. The trajectory points are aligned in the direction of the prediected orientation of the car before it is used to fit a polynomial. The way we achieve this is shown below-
```
ptsx = (ptsx - px)*cos(psi) + (ptsy - py)*sin(psi)
ptsy = -(ptsx - px)*sin(psi) + (ptsy - py)*cos(psi)
```
Here `ptsx` and `ptsy` represent the points on the trajectory. Therefore by doing this we get the initial position and orientation of the car to be zero and the polynomial is then fitted on the transformed coordinates of the planned trajectory. This results in a set of coefficients that can be used to evaluate the cross track error of the car by substituting the predicted `px` position of the car at each `N` level of the predicted movement. The order of fitting is chosen as 3.

### Model Predictve Control with latency
The Latency signifies the lag in actuator inputs with repect to the continuously changing state variables. This is implemented within the code by updating the state of the car by the same kinematic equations, but the time step is fixed as 100 milliseconds or 0.1 seconds. Without the latency, the model maneuvers the track without any problems or erroneous behaviour, but with latency, the car oscillates initially as seen in the [video](https://youtu.be/ounbYntb8rI).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

