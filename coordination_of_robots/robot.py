import numpy as np
import math
import heapq

from constants import (
    X,
    Y,
    X_GRIDS,
    Y_GRIDS,
)

class Robot():

    def __init__(self, id, pose, goal, color):
        self.id = id
        self.pose = pose #x, y, yaw
        self.d_pose = np.zeros(3) # x_dot, y_dot, theta_dot
        self.command = [0, 0] # v, w
        self.goal = goal # x, y, yaw
        self.color = color

        self.trajectory = None

        self.collision_points = []; self.collision_priority = []
        self.collision_with = [];   self.collision_impending = []

    def model(self):
        self.d_pose = np.dot(np.array([[math.cos(self.pose[2]), 0],
                                       [math.sin(self.pose[2]), 0],
                                       [0, 1]]),
                                       self.command)
        return self.d_pose

    def update_step(self, dt):
        self.pose += self.model()*dt
        self.pose[2] = math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2]))
        return self.pose

    def cmd_vel(self, v, w):
        self.command = [v, w]

    def angle_to_goal(self):
        return math.atan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0])

    def __str__(self):
        return "X: {}, Y: {}, Theta: {}".format(self.pose[0], self.pose[1], self.pose[2]*180/math.pi)
