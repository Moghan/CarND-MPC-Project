# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

A vid of the simulator running is found here - https://youtu.be/HfY3lJTSZjI

## The Model

State consist of the 6 state values, x and y coordinates, speed v, angle psi, cross-track-error cte, and angle error epsi.

Actuators are steering angle and throttle.

### Update equations
x(t+1) = x(t) + v(t) * cos(psi) * dt

y(t+1) = y(t) + v(t) * sin(psi) * dt

psi(t+1) = psi(t) + v(t) / Lf * steering_angle(t) * dt

v(t+1) = v(t) + throttle(t) * dt

cte(t+1) = f(x(t)) - y(t) + (v(t) * sin(epsi) * dt)

epsi(t+1) = psi(t) - psides(t) + (v(t) / Lf * steering_angle(t) * dt)


## Timestep Length and Elapsed Duration
Within 0.5 seconds I judge the simulator should have an update on the actuators, so the solver maximum time limit is set to 0.5 sec.
If there are to many timesteps, the solver will not come up with a good solution in time, and give erradic and car crashing values.
Increasing time (dt) between timesteps makes the horizon farther away. Timesteps * dt should not be more than a few seconds.

With trial and error, the solver seemed to give bad values with 15 timesteps. With 10 timesteps and dt = 0.2 , the horizon should only be 2 sec away,
but became to far away nonetheless. 

With timesteps = 10 and dt = 0.1 , the solver finishes in time, and the green horizon markers is stretching a desirable distans ahead.

EDIT: In the first submit, the car crashed for the reviewer. My guess is that a different processing power made the solver send less good values and making the car to crash. 
The speed is now set to 60 and solver maximum time limit is increased to 0.8 sec. This means simulator may wait 0.8 sec for new actuator values instead of 0.5, and reducing speed to 60 instead of 80 is a safety precaution due to this.

The vid is from before reducing speed and changing time limit.

## Polynomial Fitting and MPC Preprocessing
To make coordinates easier to work with, the global coordinate system origin is set to the cars position and rotated so x-axis is pointing in direction of the car.
The points (x, y) the simulator is providing are the center of the road ahead of the car, which are where we want the car to be. Coefficients for a polynomial are created with polyfit() using x and y coordinates given by the simulator as parameters.

The coefficients in combination with polyfit() and a x value gives the y value where the car is supposed to be when reaching x. If setting x = 0 when calling polyfit() it returns the 
y coordinate the car is supposed to be at presently. Since the car is origo, any other value from polyfit() than 0 means there is a CTE(cross-track-error).

CTE = polyfit( coefficients , x = 0 )

To calculate EPSI the complete formula is: 

epsi = psi - atan(coeff[1] + 2 * coeff[2] * px + 3 * coeff[3] * px *px)

but since car is origo and x-axis is in the cars direction, it can be simplified to:

epsi = -atan(coeffs[1])

In other words, if the desired angle in x=0 is not pointing in direction of x-axis, which is cars direction, there is a psi error.

When calculating CTE and epsi this way, it also shows the gain of setting car to origo and rotating coordinate system so x-axis point in direction of car.

## Model Predictive Control with Latency
With a simulated latency that is not countered, the car starts to oscillate. 

To predict a new state, to counter the latency of 0.1 sec, a kinematic model is used. Very easy to implement and it works very well.
Trying a dynamic approach is left for the future.
