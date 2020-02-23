from scipy.optimize import minimize_scalar

import numpy as np
import math


from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
    COLLISION_MARGIN,
)

from timer import Time
from plotter import Plotter
from robot import Robot
from world import World
from trajectory import Trajectory

import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

def plan(my_world, pltr, algo):
    my_world.generate_paths(algo=algo)
    for robot in my_world.robots:
        pltr.draw_start_end(robot.pose, robot.goal, robot.color)
    if my_world.obstacles:
        l, w, start = my_world.obstacles
        obstacles = [((i*0.1*l)+start[0], (j*0.1*w)+start[1]) for i in range(10) for j in range(10)]
        for obs in obstacles:
            pltr.plotting(obs[0], obs[1], 'r')
        pltr.sleep(2)
    for robot in my_world.robots:
        for point in robot.path:
            pltr.plotting(point[0], point[1], 'k')
    pltr.sleep(2)

    for robot in my_world.robots:
        robot.trajectory = Trajectory(robot.path, 9)

    makespan = max(robot.trajectory.total_time for robot in my_world.robots)

    for t in range(100):
        for robot in my_world.robots:
            if robot.trajectory.total_time >= t*makespan/100:
                x, y = robot.trajectory.location(t*makespan/100)
                pltr.plotting(x, y, robot.color, 5)
        pltr.sleep(0.001)
    pltr.sleep(2)
    pltr.close('all')
    my_world.detect_collision()

def curvature(trajectory, t):
    x_dot, y_dot = trajectory.derivative(t)
    x_ddot, y_ddot = trajectory.double_derivative(t)
    return (x_dot*y_ddot - y_dot*x_ddot)/(x_dot**2 + y_dot**2)**1.5

def execute_with_collision(my_world):
    pltr = Plotter()
    for robot in my_world.robots:
        pltr.draw_start_end(robot.pose, robot.goal, robot.color)
    makespan = max(robot.trajectory.total_time for robot in my_world.robots)
    for t in range(100):
        for robot in my_world.robots:
            if robot.trajectory.total_time >= t*makespan/100:
                x, y = robot.trajectory.location(t*makespan/100)
                pltr.plotting(x, y, 'k')

    for robot in my_world.robots:
        x_dot, y_dot = robot.trajectory.derivative(0)
        robot.pose[2] = math.atan2(y_dot, x_dot)

    completed = set()
    while True:
        for robot in my_world.robots:
            t = my_world.time.time()
            if t > robot.trajectory.total_time:
                completed.add(robot.id)
                continue

            x_dot, y_dot = robot.trajectory.derivative(t)
            vel = (x_dot**2 + y_dot**2)**0.5
            x, y = robot.trajectory.location(t)
            curv = curvature(robot.trajectory, t)
            robot.cmd_vel(vel, curv*vel)
            robot.update_step(my_world.time.dt)

            pltr.plotting(robot.pose[0], robot.pose[1], robot.color, s=5)
            pltr.mark_time(my_world.time.time())
        pltr.sleep(0.001)
        my_world.time.step()

        if completed == set([r.id for r in my_world.robots]):
            pltr.sleep(1)
            pltr.close('all')
            metric(my_world)
            break

def execute_wo_collision(my_world):
    pltr = Plotter()
    for robot in my_world.robots:
        pltr.draw_start_end(robot.pose, robot.goal, robot.color)
    makespan = max(robot.trajectory.total_time for robot in my_world.robots)
    for t in range(100):
        for robot in my_world.robots:
            if robot.trajectory.total_time >= t*makespan/100:
                x, y = robot.trajectory.location(t*makespan/100)
                pltr.plotting(x, y, 'k')

    my_world.avoid_collision()
    # print "Collision avoidance done."

    for robot in my_world.robots:
        x_dot, y_dot = robot.trajectory.derivative(0)
        robot.pose[2] = math.atan2(y_dot, x_dot)

    completed = set()
    total_time = 0
    while True:
        for robot in my_world.robots:
            t = my_world.time.time()
            if t > robot.trajectory.total_time:
                completed.add(robot.id)
                continue

            x_dot, y_dot = robot.trajectory.derivative(t)
            vel = (x_dot**2 + y_dot**2)**0.5
            x, y = robot.trajectory.location(t)
            curv = curvature(robot.trajectory, t)
            robot.cmd_vel(vel, curv*vel)
            robot.update_step(my_world.time.dt)

            pltr.plotting(robot.pose[0], robot.pose[1], robot.color, s=5)
            pltr.mark_time(my_world.time.time())
        # pltr.sleep(0.001)
        my_world.time.step()

        if completed == set([r.id for r in my_world.robots]):
            pltr.sleep(1)
            pltr.close('all')
            metric(my_world)
            break

def metric(my_world):
    print "Makespan: ", max(robot.trajectory.total_time for robot in my_world.robots)
    print "Total time: ", sum(robot.trajectory.total_time for robot in my_world.robots)
    path_len = 0
    for robot in my_world.robots:
        for dt in range(1000):
            if dt == 0:
                continue
            t = robot.trajectory.total_time*dt/1000
            t_1 = robot.trajectory.total_time*(dt-1)/1000
            dx, dy = tuple(np.subtract(robot.trajectory.location(t), robot.trajectory.location(t_1)))
            path_len += (dx**2 + dy**2)**0.5
    print "Total path length", path_len

def main():
    colors = 'bgcym'

    pltr = Plotter()

    # obstacles = [1,1,(5,4)]
    # obstacles = [1,1,(5,5)]
    obstacles = []

    print "Case 1: A-star"

    time = Time(0.01)
    # robot_1 = Robot(1, [2, 2, 0], [8, 6, 0], colors[0])
    # robot_2 = Robot(2, [8, 2, 0], [2, 6, 0], colors[1])
    robot_1 = Robot(1, [5, 9, 0], [5, 1, 0], colors[1])
    robot_2 = Robot(2, [1, 6, 0], [7, 6, 0], colors[2])

    my_world = World([robot_1, robot_2], time, obstacles)

    plan(my_world, pltr, "a-star")
    # execute_with_collision(my_world)
    execute_wo_collision(my_world)

    print '----------------\n'

    del robot_1
    del robot_2
    del my_world

    pltr = Plotter()
    print "Case 2: RRT"
    time = Time(0.01)

    # robot_1 = Robot(1, [2, 2, 0], [8, 6, 0], colors[0])
    # robot_2 = Robot(2, [8, 2, 0], [2, 6, 0], colors[1])
    # robot_3 = Robot(3, [4, 4, 0], [8, 8, 0], colors[3])
    # robot_4 = Robot(4, [5, 6, 0], [6, 8, 0], colors[4])
    robot_1 = Robot(1, [5, 9, 0], [5, 1, 0], colors[1])
    robot_2 = Robot(2, [1, 6, 0], [7, 6, 0], colors[2])

    my_world = World([robot_1, robot_2], time, obstacles)

    plan(my_world, pltr, "rrt")
    # execute_with_collision(my_world)
    execute_wo_collision(my_world)

if __name__ == "__main__":
    main()
