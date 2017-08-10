# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Model Predictive Control with Latency
With a simulated latency that is not countered, the car starts to oscillate. 

To predict a new state, to counter the latency of 0.1 sec, a kinematic model is used. Very easy to implement and it works very well.
Trying a dynamic approach is left for the future.