# -*- coding: utf-8 -*-

from math import atan2, radians, cos, sin, pi
from random import gauss

from ts import distance

def NAO_turn_forward(cur_pose, goal, time_step=1, theta_tol=0.4, x_speed=50, turn_speed=0.2):
    # cur_pose = (x, y, orientation), orientation in radian
    # from tf2pose
    delta_x = goal[0] - cur_pose[0]
    delta_y = goal[1] - cur_pose[1]
    theta_ref = atan2(delta_y, delta_x) # in radian
    dist = distance([delta_x, delta_y], [0,0])
    theta_dif = theta_ref - cur_pose[2]
    next_pose = [0, 0, 0]
    if abs(theta_dif)< theta_tol: # forward
        #print 'forward' x_speed, (0.17*dist+15)
        next_pose[0] = cur_pose[0] + x_speed*time_step*cos(cur_pose[2])+ 1*gauss(0,5)
        next_pose[1] = cur_pose[1] + x_speed*time_step*sin(cur_pose[2])+ 1*gauss(0,5)
        next_pose[2] = cur_pose[2] + 1*gauss(0,0.0005)
    else: # turn 
        #print 'turn'
        next_pose[0] = cur_pose[0] + 1*gauss(0,5)
        next_pose[1] = cur_pose[1] + 1*gauss(0,5)
        dif_sign = theta_dif/(abs(theta_dif)+0.00001)
        if abs(theta_dif)<= pi:
            # theta_dif
            next_pose[2] = cur_pose[2] + turn_speed*(dif_sign)*time_step + 1*gauss(0, 0.0005)
        else:
            # (2*pi-abs(theta_dif))
            next_pose[2] = cur_pose[2] - turn_speed*(dif_sign)*time_step + 1*gauss(0, 0.0005)
    # convert theta to [-pi, pi]
    if next_pose[2]> pi:
        next_pose[2] = next_pose[2] - 2*pi
    elif next_pose[2]< -pi:
        next_pose[2] = next_pose[2] + 2*pi
    #print next_pose
    return next_pose    




