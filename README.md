# Vehicle Dynamics Simulation
## intro
In this file you have a code which move a vehicle by use the Pure pursuit tracking.
The input is the initial position and the output is a figure of the desired and the real paths of the vehicle and the reference point of each epoch.
Each epoch represnt a time step of the movment.

In addtion, you have the file of the theoretical questions

## code explantion
### main
The logical way the main works is as follows: receiving input, indexing for the desired path (what the vehicle actualy need to follow), after a while loop which update every state of the vehicle and each step printing an epoch of the state (print a figure of the global coordinate system),the task is finished when we assign to the end of the path or the  time runs out.  

At first we get as a input from the user a initial point to start. The main plan the route (line, circle etc.) simulation max time and the veicleState class. 

### VehicleState
VehicleState is a class that difine the vehicle state information

### update
the function update is the update that occurs every time step. Int is the Forward Euler method.

### pure_pursuit_control
the function execute the traking algoritem.

### calc_target_index
Find the nearest point on the desired path and output the position of the nearest point







