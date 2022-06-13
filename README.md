# Vehicle Dynamics Simulation
## Introduction 
This repository contains a code that moves a vehicle using Pure Pursuit tracking.
The input is the initial position, and the output is a figure of both the desired and the real paths of the vehicle, as well as the reference point of each epoch. Each epoch represents a time step in the movment.

You can also find the file for the theoretical questions

## code explanation 
To execute the code run in terminal: `python3 assignment_car.py x0 y0 yaw0 v`

Where x0 and y0 are the initial coordinates, yaw0 is the initial orientation and v is the constant speed of the vehicle. 

The main simulates a 2D vehicle that tracks a simple geometric 2D path. As the desired path is created, the vehicle moves to the nearest point on the path (using the methods below) until it reaches the end of the path or the simulation time runs out. 

### Simulation Example

the input: `0.1 0.1 0.0 1.0` 

the output:

https://user-images.githubusercontent.com/103378199/173420215-1f0cc59b-3246-4ec4-95e0-c90c7a8688f7.mov



### VehicleState class
VehicleState is a class that defines the vehicle state information (x,y position, yaw angle, velocity)   

### update method
Updates the vehicle state information in each time step. 

### pure_pursuit_control method 

The tracking algorithm finds the point on the path the vehicle needs to move to and sends a reference signal to the controller regarding how much steer change needs. The controller tries to move the vehicle in the right direction according to the some limitations as rate change and max value of the steer angle.

### second_order_Control method

This method is the controller. I decided on the bandwidth and the damping coefficient from the internet. 

### calc_target_index method
Return the position of the nearest point found along the desired path.








