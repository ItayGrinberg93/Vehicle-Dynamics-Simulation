# assignment_for_mobileye
## intro
In this file you have a code which move a vehicle by use the Pure pursuit tracking.
The input is the initial position and the output is a figure of the desired and the real paths of the vehicle and the reference point of each epoch.
Each epoch represnt a time step of the movment.

In addtion, you have the file of the theoretical questions

## code explantion
### main
The logical way the main works is as follows: receiving input, indexing for the desired path (what the vehicle actualy need to follow), the while loop which update every state of the vehicle, and each step the main print an epoch of the state (print a figure of the global coordinate system),the task is finished when time runs out or we assign to the end of the path.  

At first we get as a input from the user a initial point to start. The main plan the route (line, circle etc.) simulation max time and the veicleState class. 









