#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys


# Define constant
k = 0.1  # look forward gain
lad = 1  # look - ahead distance
dt = 0.5 # [ s ] time step size
L = 2.85 # [ m ] wheelbase of vehicle
Kp=12# propotional gain
w=80 # bandwidth
zeta=0.5 # damping
delay = 0.2 # [sec]




class VehicleState: # Define a class to call vehicle state information

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


# step
def update(state, delta):  # Update vehicle status information
    """
           2D Kinematic Bicycle Model
           At initialisation
           :param L:                   (float) vehicle's wheelbase [m]
           :param dt:                  (float) discrete time period [s]

           At every time step
           :param x:                   (float) vehicle's x-coordinate [m]
           :param y:                   (float) vehicle's y-coordinate [m]
           :param yaw:                 (float) vehicle's heading [rad]
           :param v:                   (float) vehicle's velocity in the x-axis [m/s]
           :param delta:               (float) vehicle's steering angle [rad]
           :param angular_velocity     (flaat) V*tan(delta) / L [rad/sec]
    """
    noise = np.random.normal(0, .05, 3) # white noise
    # position integration
    state.x = state.x + state.v * np.cos(state.yaw) * dt + noise[0]
    state.y = state.y + state.v * np.sin(state.yaw) * dt + noise[1]
    state.yaw = state.yaw + state.v / L * np.tan(delta) * dt + noise[2]
    return state


# Second order
def second_order_control(u, u_p):
    a = Kp*w**2 * (u-u_p) * (dt+delay)**2 + 2 * zeta * w * u_p * (dt+delay)
    return a


# Pure tracking controller , Sets the  angular change
def pure_pursuit_control(state, cx, cy, delta_previous, pind):

    ind = calc_target_index(state, cx, cy)  # Find the function of the nearest point and output the position of the nearest point

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = np.arctan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # reverse
        alpha = np.pi - alpha

    lf = k * state.v + lad
    delta = np.arctan2(L * np.sin(alpha), lf)
    delta = second_order_control(delta, delta_previous)
    # limits on delta rate
    if abs(delta - delta_previous) > 20 * np.pi / 180:
        if delta > delta_previous:
            delta = delta_previous + 20 * np.pi / 180
        else:
            delta = delta_previous-20 * np.pi / 180
    # limits on delta
    if delta > 45 * np.pi / 180:
        delta = 45 * np.pi / 180
    if delta < -45 * np.pi / 180:
        delta = -45 * np.pi / 180
    return delta, ind


def calc_target_index(state, cx, cy):
    # Find the serial number of the closest point to the current position of the vehicle

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(np.sqrt(idx**2 + idy**2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0
    lf = k * state.v + lad

    # Search look Ahead target Point index
    while lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += np.sqrt(dx**2 + dy**2)
        ind += 1

    return ind


def main():
    # options to Routes

    # line
    # cx = np.arange(0, 50, 0.1)
    # cy = [ ix*1+0.5 for ix in cx]

    # circle Routes
    cx = 10*np.sin(np.arange(0, 2*np.pi, 0.01))
    cy = 10*np.cos(np.arange(0, 2*np.pi, 0.01))

    # Custom Routes
    # df = pd.read_csv(r'Path where the CSV file is stored\File + name.csv') #read the csv file
    # cx = df["cx"]
    # cy = df["cy"]

    T = 300  # max simulation time [sec]

    # Initial State
    state = VehicleState(x=float(x0), y=float(y0), yaw=float(yaw0), v=float(v0))

    lastIndex = len(cx) - 1

    time = 0
    x = [state.x] # x position
    y = [state.y] # y position
    yaw = [state.yaw] # angular velocity
    v = [state.v] # linear velocity
    t = [0]
    di = 0
    target_ind = calc_target_index(state, cx, cy)

    # Continuously perform update operations
    while T >= time and lastIndex > target_ind:
        di, target_ind = pure_pursuit_control(state, cx, cy, di, target_ind) # angular velocity change
        state = update(state, di)
        time = time + dt  # update time

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        plt.cla()
        plt.plot(cx, cy, ".r", label="desired path")  # desired path
        plt.plot(x, y, "-b", label="real path") # route
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="reference point") # reference point
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[: 4])
        plt.xlabel("x location")
        plt.ylabel("y location")
        plt.legend(loc='upper right')
        plt.pause(0.001)
        # plt.show()


if __name__ == '__main__':
    try:
        x0 = sys.argv[1]
        y0 = sys.argv[2]
        yaw0 = sys.argv[3]
        v0 = sys.argv[4]
        print("Pure pursuit path tracking simulation start")
        main()

    except NameError:
        print("Code not found")
        raise NotImplementedError

