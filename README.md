# CarND-Controls-MPC
## Self-Driving Car Engineer Nanodegree Program

# Intro
#### This project implements a Model Predictive Controller (MPC) to control a vehicle smoothly around a track. The simulator provides the vehicle position (x, y), speed and heading angles. The Model Predictive Controller take those variable as input and return a predict of the future states.

# Model 
#### The Model take in the following variable from the simulator about the vehicle:

* px - Px is the global map coordinate x of the current location of the vehicle
* py - Py is the global map coordinate y of the current location of the vehicle
* psi - the current heading angle/ direction heading of the vehicle
* v - the current speed/ velocity of the vehicle 

#### We also give a vector of waypoints 
* ptsx - x of waypoints respect to global map coordinate
* ptsy - y of waypoints respect to global map coordinate


#### we are going to use this waypoints to fit into 3rd degree polynomial equation to predict and estimates the curve of the road. Before we fit the waypoint into the polynomial equation, we need to change the global map coordiante waypoint to the vehicle coordiante.

```
x_waypoint[i] = tran_x * cos(-psi) - tran_y * sin(-psi);
y_waypoint[i] = tran_x * sin(-psi) + tran_y * cos(-psi);
```

#### After change the map coordiante repect to vehicle coordiante, we can use the ployfit equation to fit the 3rd degree polynomial equation, since 3rd degree polynomial equation is enough to estimate most of the road curves.

```
Eigen::VectorXd fit_coeff = polyfit(x_waypoint, y_waypoint, 3);
```
#### We calcuate the current state and adjust the latency using the following equation:
```
double current_dt = 0.1; // 100 ms
double Lf = 2.67;
double current_x = v * current_dt;
double current_y = 0;
double current_psi = -(v / Lf) * steering_angle * current_dt;
double current_v = v + throttle * current_dt;
double current_cte = cte + v * sin(epsi) * current_dt;
```
#### The steering_angle and throttle is the variable from the simulator and Lf is the distance between the front of the vehicle and centre gravity of the vehicle. dt is the latency for the simulator. 

#### We input the result of the polyfit and the current state into the Solve function. This function will return actuators: new steering angle and new throttle.
```
vector<double> solution = mpc.Solve(state, fit_coeff);
double steer_value = solution[0];
double throttle_value = solution[1];
```
#### The goal is that the MPC will control the vehicle drive inside the track and does not need to brake too hard when the road is curve. 

## Idea of the Cost function and penalty weight 
#### The function will predict the best movement that minimum the cost. The Cost are the sum of All Cost(penalty multiply to each variable) for each N (number of point we want to predict).  I put different penalty weight on different variable like the following:
```
const double w_cte = 1000.0;
const double w_epsi = 5000.0;
const double w_v = 1.0;
const double w_delta = 3000;
const double w_a = 10;
const double w_diff_delta = 5000;
const double w_diff_a = 10000;
```
#### The reason to choose those weight is I do not want the vehicle change the angle and speed to quick, so I put highest penalty weight on the changing acceleration, second is the changing angle and epsi(orientation error). I put more penalty weight on those variable, so the cost function will be more sensitive about those variable, and will produce nice prediction line for the vehicle to drive.

#### After that we can test with N and dt with different, and I choose this at the end:
```
const size_t N = 9;  // when N is greater then 10, at same turn, it show a non smooth line.
					 // cause by calculate time is not enough, if set N =15, it will show up more time. 
const double dt = 0.1;
```
#### I try with different number for N, and I end up with N = 9. the reason for 9 is beacuse it is the maximum number that is smooth curvy line. If the number is increase to 10, on one of the turn, the prediction line will lead to the outside of the track for a second. Like this:
![MPC](https://github.com/gon1213/MPC_control/blob/master/image/wrong_line.png)
#### If the number is increase more, it will show up more place and more time. The reason for that is cause by calculate time too long, not finish the calculation before the next input. That is also the reason to choose dt = 0.1, is dt is smaller, the N value need to decrease to let the calculation finish in time. On the other hand, we do not want ot increase the dt too much just for the calculation, because that will shorter the reaction time for the curve. This is trade off between the computation time and the number we can predict of the future. 

## Prediction
#### For the prediction, I implement the ipopt to calculate the the best line respected to the penalty weight and return the minimize the cost. And produce The prediction line as green in the following image.

![MPC](https://github.com/gon1213/MPC_control/blob/master/image/correct.png)


