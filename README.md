# PID Controller
This is my submission for Udacity Self-Driving Car Engineer Nanodegree Program's PID controller project.

---

# Description of the P, I, D components

The heart of the program is a PID controller. Its components are:
- Proportional component: it is responsible for steering the car back to the center. Steering angle is proportional with the distance from the center.
- Integral component: is able to remove the effect of a constant force, e.g. wind or gravity on a crooked road. These effects were not expected to be present in the simulator, so I did not expect this component to be useful in this case. I was proven wrong. See below.
- Differential component: is able to reduce the waving back-and forth aroung the center of the road by taking into account the speed of change.

Each of those components have their own coefficients (Kp, Ki, Kd), which needed to be set up properly. For details see below.
I measured the car distances from the center of the track (i.e. cross-track error, CTE) while tweeked the coefficients.

## Lessons learned
I expected that the ideal Ki value would be 0. I found that a small positive Ki was better (0.002). My guess is that it is so because the track goes counter-clockwise, and out of 6 turns 5 is a left turn, and that coefficient helps in these turns keeping closer to the center-line.

## Speed
I also added another PID controller that keeps a given speed. Then I increased the speed until I found a good balance between speed and smoothness. The maximum speed I could get  around the lap with was 60MPH, but that worked only rarely and the drive was far from smooth. So I went back to 45MPH. With this speed the car can circle forever without falling down with a decent smoothness.

# Choosing the final parameters
First I experimented with manual parameter tweeking. My method was:
- Start with a tiny Kp, and other coefficients set to zeroes.
- If the car is not turning fast enough: increase Kp.
- If the car is waving back-and forth around the center line: increase Kd.
- Keep Ki at zero.

After some experimentation I started to increase the speed of the car. I found that after each speed increase I need to find new coefficients, as the old ones did not work well anymore.

Then I changed my method and implemented the Twiddle algorithm.

## Twiddle
My algorithm is based on Udacity Self-Driving Car nanodegree program, Lession 12/14 as presented by Sebastian, but with one major difference (and lots of smaller ones). 

### Major difference: order of parameter tweaking
The original algorithm tweaked the first coefficient again and again thousands of times, and it moved forward to the next parameter just after the first parameter was considered final. The current implementation, in turn, works like this:
- Try tweaking one coefficient once up and if needed down.
- Measure the goodness, then modify the tuning parameter that belongs to that coefficient (e.g. tuning_param[0] for Kp, tuning_param[1] for Ki, etc.).
- Move to the next parameter immediately.
  - If there are no more parameters: move to the first parameter in order: Kp -> Ki -> Kd -> Kp...
- Repeat until an exit condition is met.

#### Exit if at least one of these condition is met:
- All tuning parameters are below the preset goal values. This means means that we are very close to the optimal coefficients.
- Max CTE is less than 1.5 metes, which means the car goes very smoothly.
- CTE is bigger than 5.5 meters, which means the car felt off the road.

#### Measuing of the goodness:
I measured the absolute values of average CTE and max CTE. Then applied weights of 1.0 and 9.0 to express that keeping the car on track is much more important than smooth driving.

I used the above algorithm to fine tune the coefficients. From time to time I stopped to improve it. At these occasions I copied the coefficents from stdout that belonged to the best run and set those values as new starting values for in main.cpp. 

### Lessons learned
By using Twiddle algorithm I found the following:
- The algorithm works. It definitely helps smoothing the ride.
- After increasing the speed manually the previously found coefficients are not really useful anymore.

### What could be improved
If the goal is to find the highest speed and the coefficients for it, then the current algorithm is suboptimal. Idea to improve for this purpose:
- Let the car fall off the road but measure the driven distance before that happens.*
- Add that distance driven before falling down to the goodness function.
- Reset the car.
- Repeat until the car cannot go a full lap within given smoothness limits.
- Increase the speed a tiny bit and repeat.

*Note: the current algorithm doesn't work well if the car falls off the road. It need decent starting values to be able to do a full lap, thus allowing for measurements of average and max CTEs.



---

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

