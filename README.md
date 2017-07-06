# CarND-Controls-MPC
## Self-Driving Car Engineer Nanodegree Program

# Intro
## This project implements a Model Predictive Controller (MPC) to control a vehicle smoothly around a track. The simulator provides the vehicle position (x, y), speed and heading angles. The Model Predictive Controller take those variable as input and return a predict of the future states.

# Model 
## The Model take in the following variable from the simulator about the vehicle:

### px - Px is the global map coordinate x of the current location of the vehicle
### py - Py is the global map coordinate y of the current location of the vehicle
### psi - the current heading angle/ direction heading of the vehicle
### v - the current speed/ velocity of the vehicle 

## We also give a vector of waypoints 
### ptsx - x of waypoints respect to global map coordinate
### ptsy - y of waypoints respect to global map coordinate


## we are going to use this waypoints to fit into 3rd degree polynomial equation to predict and estimates the curve of the road. Before we fit the waypoint into the polynomial equation, we need to change the global map coordiante waypoint to the vehicle coordiante.

```
x_waypoint[i] = tran_x * cos(-psi) - tran_y * sin(-psi);
y_waypoint[i] = tran_x * sin(-psi) + tran_y * cos(-psi);
```

## After change the map coordiante repect to vehicle coordiante, we can use the ployfit equation to fit the 3rd degree polynomial equation, since 3rd degree polynomial equation is enough to estimate most of the road curves.

```
Eigen::VectorXd fit_coeff = polyfit(x_waypoint, y_waypoint, 3);
```
