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

The tracking algorithm determines where the vehicle needs to move and sends a reference signal to the controller that indicates how much steering change is required. The controller attempts to move the vehicle according to some limitations such as rate change and maximum steer angle.

### second_order_Control method

This method is the controller. The first value of the bandwidth and damping coefficient were chosen based on literature and some itrations.  

### calc_target_index method
Return the position of the nearest point found along the desired path.








