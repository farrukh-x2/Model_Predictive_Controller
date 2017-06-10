# CarND-Controls-MPC
From Self-Driving Car Engineer Nanodegree Program, Starter Code Provided by Udacity

![](MPC.gif)


#### Inputs to Executable file
Passing N dt and ref_v values at runtime is optional. If no values are passed the best ones will be used.

---

## Implementation
#### The Model
The model of the car is based on the kinematic model. The states are position in X and Y, orientation, velocity, error in desired orientation and cross track error. The actuators are throttle (from -1 to 1) and steering angles (-25 degrees to 25 degrees).
The update equations of the model are given below (implemented in mpc.cpp):

      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

#### Timesteps and Duration
For parameter tunning, I first started with dt equal to simulated latency of actuators (0.1). For large values of N (>15) the car would not track well. Reducing the N to within 6-8 produced better results. Then dt was tuned again. Increasing dt from 0.1 to 0.15 would make the car go off track near the first turn. Reducing the value too much made the car oscillate. Once the best value of dt was found, N was adjusted again.

The above procedure was repeated for until a safe speed (ref_v) of 60 mph was attained. The car can drive with speeds up to 75 mph, but at  speeds greater than 60 mph the tire might occasionally pop up on the ledge. The speed in the gif file is 70 mph.

#### Preprocessing and Polynomial Fitting
The waypoints are transformed from the map coordinates to the car coordinates system. This transformation is implemented as below:

    double transf_x, transf_y;
    transf_x =     (wayp_x- car_x)*cos(car_theta) + (wayp_y- car_y)*sin(car_theta);
    transf_y =  -1*(wayp_x- car_x)*sin(car_theta) + (wayp_y- car_y)*cos(car_theta);


A second degree polynomial is fitted to the trans waypoints. The coefficients of the which are used to generate the reference points. These reference points are shown by the yellow line in the gif.

#### Latency
First I tried to adjust for latency by predicting the state variable values 0.1 seconds in the future and passing those as input to the solver. This didn't result in a big improvement from my current model. The current model can deal with latency because it calculates multiple values into the time horizon and a 0.1 second latency doesn't create a significant impact.


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



The `lake_track_waypoints.csv` file has the waypoints of the lake track.
