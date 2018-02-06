# Project: PID Controller
## Overview
This project is about using a [Proportional Integral Derivative controller(PID)](https://en.wikipedia.org/wiki/PID_controller)  for controlling car steering angle & throttle.


## How Does It Work?

The PID controller works on the principal of counteracting against the cross track error (CTE) or simply the error, which is roughly the distance from the point where the vehicle should ideally be located to the current location of the vehicle. So basically the more CTE is there, the more steering angle correction is applied (counter steer).

## Rubric Points

## Reflection
#### Describe the effect each of the P, I, D components had in your implementation.

* **P** The P (proportional) component is inversely proportional to the CTE i.e. the greater the error, the more aggressive this component is by counteracting against the error. However, just by using this component alone, the car swerves badly which is due to the reason that by the time the car compensates for the error, it overshoots to the other side of the reference point at which point it tries to correct itself again and then the whole cycle repeats itself over and over again (oscillatory motion) as shown in [this video clip](/data/P.mp4). Increasing P causes an increase in oscillations and vice-versa.

* **D** The D (derivative) component is the rate of change of the CTE i.e. its derivative (difference between current and previous CTE). It takes into consideration the previous CTE and reduces the oscillatory movement by reducing the aggressive correction applied by the P component as shown in [this video clip](/data/PD.mp4). Basically, as the error rate decreases, it reduces the effect of P and vice-versa.

* **I** The I (integral) component is sum of all the previous components. It is generally employed when there is a systematic bias in the system e.g. wheel misalignment, lateral slope for drainage, etc. and  works on this intuition that if the overall error is on the rise then it applies more correction (on either direction) and vice-versa. The resulting vehicle movement after the application of PID components can be seen in [this video clip](/data/PID.mp4). 

The below graph shows the impact of the application of each component:

![PID](/data/pid.png)


### Describe how the final hyper parameters were chosen.

I started by implementing Twiddle for the steering angle PID controller’s parameters (tau). I first tried by optimizing all 3 parameters with the default throttle of `0.3`. However, this didn’t work as the car would crash within few seconds. Then the throttle was reduced to `0.1` as there’s not enough straight track for the Twiddle to converge. However, that still didn’t work. Based on the lessons, I changed the order of parameter optimization so that the order is [P,D,I] but it seemed like the I parameter was causing the car to crash quite early on. It could be that the required I parameter value was very small but Twiddle started off with 1 and required considerable amount of steps (reporting of CTE by the simulator, calculating the steering angle & sending the calculated angle back to the simulator = 1 step). Consequently, only the P & D parameters were kept for optimization (I was kept as `0`).The optimization seemed to generate some viable values for P & D but only for throttle below `0.2`. It is important to mention that I configured the Twiddle to run continuously if the Twiddle error is above a threshold or the CTE is above `0.0029` (observed value when the car is going straight without oscillating).

Having spent a lot of time trying to use Twiddle for parameter optimization, I decided to manually tune the parameters by resorting to use the values introduced in the lesson as the starting point (`P=0.2,D=3.0,I=0.004`). I used the *extreme angle count* as a statistic to gauge *ride smoothness*. Below is a summary of the results:

| P | D | I | Extreme Angle count     		| 
|:--:|:--:|:--:|--:| 			
| 0.2 | 3.0 | 0.004 | 7 |
| 0.4 | 6.0 | 0.004 | 27 |
| 0.6 | 9.0 | 0.004 | 88 |
| 0.2 | 6.0 | 0.004 | 18 |
| 0.2 | 4.0 | 0.004 | 11 |

I kept the I constant `0.004` in the start and concentrated on optimizing P & D. Further, I kept the same ratio (`1:15`) to begin with. Generally, increasing the parameters’ size resulted in roughly exponential increase in extreme angles. On the other hand, just increasing P or D resulted in more extreme angles. However, in case of P it made the car slow to respond when encountered with curves. Also, decreasing P or D in the same ratio, made the car very slow to correct itself, e.g. with a setting of `P=0.1` & `D=1.5`, the crashed straightaway. Similarly, decreasing D from `3` to `2.0` & `1.5` resulted in very slow correction, leading to a crash. In the end, I settled for `P=0.2` & `D=4.0`.

Next, I looked into optimizing the I value by keeping P & D constant (`0.2` & `4.0`). Setting I to `0` from `0.004`caused the car to go very slow, which is suggestive of large CTE. The results of other tested values for I are listed below:

| P | D | I | Description   		| 
|:--:|:--:|:--:|-------------------------------:| 			
| 0.2 | 4.0 | 0.1 |Too much correction, straightaway crash|
| 0.2 | 4.0 | 0.01 |A bit better but still crashed|
| 0.2 | 4.0 | 0.001 |Worked but extreme correction & slow to react i.e. touched the edges few times but no crash|
| 0.2 | 4.0 | 0.0001 |Worked with fewer extreme angles but a bit slow to correct on curves|
| 0.2 | 4.0 | 0.0005 |Still hitting the curb|
| 0.2 | 4.0 | 0.005 |Much better but extreme angles increased to 30|
| 0.2 | 4.0 | 0.003 |Same as 0.0004|
| 0.2 | 4.0 | 0.007 |Hitting the curb quite early on|

Finally, I chose to use the values `P=0.2`, `D=4.0` & `I=0.004` for the steering PID controller as these values provided a smoother drive yet the car was reactive enough on curves in terms of timely steering correction.
 
**Findings:**

1. Optimize parameters in this order : P -> D -> I
2. For best results, the parameters need to be optimized constantly taking into account speed, road curvature, friction, rain, snow, wind, lateral slope & uphill/downhill factors. 
3. Different parameter values for different throttle/speed values as at higher speeds a sharper steering correction is required else the car ends up hitting the curb.
4. The evaluation steps (number of steps before Twiddle evaluates error) need to be reduced for higher speeds and vice-versa.

Apart from the steering PID controller, I also implemented a throttle PID controller and was able to use Twiddle for optimizing the parameters. The throttle PID controller is in fact configured to constantly use Twiddle. The effect of throttle PID controller resulted in automatically slowing down the car at curves and speeding it up at straight patches. 


The below graph shows the impact of the application of each Twiddle in relation to the PID:

![PID](/data/pid_twiddle.png)

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Usage

### With Simulator

Follow the build instructions above. Once the program is running, start the simulator. You should see a *connected!!!* message upon successful connection between the simulator and the c++ program. Hit the *Start button*. 

## Directory Structure

* **data:** Directory containing video clips and images
* **src:** Directory containing c++ source files
* **CMakeLists.txt:** File containing compilation instructions
* **README.md:** Project readme file
* **install-mac.sh:** Script for installing uWebSockets on Macintosh
* **install-ubuntu.sh:** Script for installing uWebSockets on Ubuntu

## License

The content of this project is licensed under the [Creative Commons Attribution 3.0 license](https://creativecommons.org/licenses/by/3.0/us/deed.en_US).