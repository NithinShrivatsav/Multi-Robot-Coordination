from scipy.optimize import minimize_scalar

import numpy as np
import math

from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
)

from timer import Time
from plotter import Plotter
from robot import Robot
from world import World
from trajectory import Trajectory

def plan(my_world, pltr):
    my_world.generate_paths()
    for robot in my_world.robots:
        pltr.draw_start_end(robot)
    pltr.sleep(2)
    for robot in my_world.robots:
        for point in robot.path:
            pltr.plotting(point[0], point[1], 'k')
    pltr.sleep(2)

    TIME = 2
    MAX_VEL = 5
    makespan = 0
    for robot in my_world.robots:
        x = [p[0] for p in robot.path]
        y = [p[1] for p in robot.path]
        t = np.linspace(0, TIME, len(robot.path))
        p = np.polyfit(t, zip(x, y), 3)
        poly1d_x = np.poly1d(p[:,0])
        poly1d_y = np.poly1d(p[:,1])

        dx = np.polyder(poly1d_x)
        dy = np.polyder(poly1d_y)
        velocity_sq = dx * dx + dy * dy

        duration = TIME
        res = minimize_scalar(-1 * velocity_sq, bounds=(0, TIME), method='bounded')
        max_velocity = velocity_sq(res.x)**0.5

        duration = TIME * max_velocity/MAX_VEL
        if duration > makespan:
            makespan = duration
        scaling_polynomial = np.poly1d([MAX_VEL/max_velocity, 0])
        poly1d_x = np.polyval(poly1d_x, scaling_polynomial)
        poly1d_y = np.polyval(poly1d_y, scaling_polynomial)
        dx = np.polyder(poly1d_x)
        dy = np.polyder(poly1d_y)
        robot.trajectory = [duration, poly1d_x, poly1d_y, dx, dy]

    for t in range(100):
        for robot in my_world.robots:
            # print robot.trajectory[1](t*makespan/1000), robot.trajectory[2](t*makespan/1000)
            if robot.trajectory[0] >= t*makespan/100:
                pltr.plotting(robot.trajectory[1](t*makespan/100), robot.trajectory[2](t*makespan/100), robot.color, 5)
        # pltr.sleep(0.001)
    pltr.sleep(2)
    pltr.close('all')
    # my_world.detect_collision()

def curvature(x, y, t):
    x_dot = np.polyder(x);  y_dot = np.polyder(y)
    x_ddot = np.polyder(x_dot); y_ddot = np.polyder(y_dot)
    return (x_dot(t)*y_ddot(t) - y_dot(t)*x_ddot(t))/(x_dot(t)**2 + y_dot(t)**2)**1.5

def execute_with_collision(my_world):
    pltr = Plotter()
    for robot in my_world.robots:
        pltr.draw_start_end(robot)
    makespan = max(robot.trajectory[0] for robot in my_world.robots)
    for t in range(100):
        for robot in my_world.robots:
            if robot.trajectory[0] >= t*makespan/100:
                pltr.plotting(robot.trajectory[1](t*makespan/100), robot.trajectory[2](t*makespan/100), 'k')

    for robot in my_world.robots:
        robot.pose[2] = math.atan2(robot.trajectory[4](0), robot.trajectory[3](0))
        print robot.pose[2]
    exit()
    completed = set()
    while True:
        for robot in my_world.robots:
            del_x = robot.goal[0] - robot.pose[0]
            del_y = robot.goal[1] - robot.pose[1]
            rho = math.sqrt(del_x**2 + del_y**2)

            if abs(rho) < 0.25:
                completed.add(robot.id)
                continue

            t = my_world.time.time()
            vel = (robot.trajectory[3](t)**2 + robot.trajectory[4](t)**2)**0.5
            curv = curvature(robot.trajectory[1], robot.trajectory[2], t)
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
        pltr.draw_start_end(robot)
    makespan = max(robot.trajectory[0] for robot in my_world.robots)
    for t in range(100):
        for robot in my_world.robots:
            if robot.trajectory[0] >= t*makespan/100:
                pltr.plotting(robot.trajectory[1](t*makespan/100), robot.trajectory[2](t*makespan/100), 'k')

    path_len = 0
    for robot in my_world.robots:
        for dt in range(1000):
            if dt == 0:
                continue
            t = robot.trajectory[0]*dt/1000
            t_1 = robot.trajectory[0]*(dt-1)/1000
            dx = robot.trajectory[1](t) - robot.trajectory[1](t_1)
            dy = robot.trajectory[2](t) - robot.trajectory[2](t_1)
            path_len += (dx**2 + dy**2)**0.5
    # print "Total path length", path_len

    for robot in my_world.robots:
        robot.pose[2] = math.atan2(robot.trajectory[4](0), robot.trajectory[3](0))
    completed = set()

    total_time = 0
    while True:
        for robot in my_world.robots:
            del_x = robot.goal[0] - robot.pose[0]
            del_y = robot.goal[1] - robot.pose[1]
            rho = math.sqrt(del_x**2 + del_y**2)

            if abs(rho) < 0.25:
                if robot.id not in completed:
                    completed.add(robot.id)
                    total_time += my_world.time.time()
                continue

            t = my_world.time.time()
            for pt_id, pt in enumerate(robot.collision_points):
                # print ((robot.pose[0] - pt[0])**2 + (robot.pose[1] - pt[1])**2)**0.5
                if ((robot.pose[0] - pt[0])**2 + (robot.pose[1] - pt[1])**2)**0.5 < 1:
                    print robot.id, " has changed its speed."
                    if robot.collision_priority[pt_id] == 'inc':
                        scaling_polynomial = np.poly1d([2, -t])
                    else:
                        scaling_polynomial = np.poly1d([0.5, t/2])
                    robot.trajectory[1] = np.polyval(robot.trajectory[1], scaling_polynomial)
                    robot.trajectory[2] = np.polyval(robot.trajectory[2], scaling_polynomial)
                    robot.trajectory[3], robot.trajectory[4] = np.polyder(robot.trajectory[1]), np.polyder(robot.trajectory[2])
                    del(robot.collision_points[pt_id])
                    del(robot.collision_priority[pt_id])
                    # del(robot.collision_with[pt_id])

            if len(robot.collision_points) != len(robot.collision_impending):
                r2 = robot.collision_with[0]
                if ((r2.pose[0] - robot.pose[0])**2 + (r2.pose[1] - robot.pose[1])**2)**0.5 > 2.3:
                    print robot.id, " is back to its original speed."
                    if robot.id < r2.id:
                        scaling_polynomial = np.poly1d([0.5, t/2])
                    else:
                        scaling_polynomial = np.poly1d([2, -t])
                    robot.trajectory[1] = np.polyval(robot.trajectory[1], scaling_polynomial)
                    robot.trajectory[2] = np.polyval(robot.trajectory[2], scaling_polynomial)
                    robot.trajectory[3], robot.trajectory[4] = np.polyder(robot.trajectory[1]), np.polyder(robot.trajectory[2])
                    del(robot.collision_impending[0])
                    del(robot.collision_with[0])

            x_der = np.polyder(robot.trajectory[1]); y_der = np.polyder(robot.trajectory[2])
            vel = (x_der(t)**2 + y_der(t)**2)**0.5
            curv = curvature(robot.trajectory[1], robot.trajectory[2], t)
            robot.cmd_vel(vel, curv*vel)
            robot.update_step(my_world.time.dt)

            pltr.plotting(robot.pose[0], robot.pose[1], robot.color, s=5)
            pltr.mark_time(my_world.time.time())
        pltr.sleep(0.001)
        my_world.time.step()

        if completed == set([r.id for r in my_world.robots]):
            pltr.sleep(1)
            pltr.close('all')
            print "Makespan: ", my_world.time.time()
            print "Total time: ", total_time
            print "Total path len: ", path_len
            break

def metric(my_world):
    print "Makespan: ", max(robot.trajectory[0] for robot in my_world.robots)
    print "Total time: ", sum(robot.trajectory[0] for robot in my_world.robots)
    path_len = 0
    for robot in my_world.robots:
        for dt in range(1000):
            if dt == 0:
                continue
            t = robot.trajectory[0]*dt/1000
            t_1 = robot.trajectory[0]*(dt-1)/1000
            dx = robot.trajectory[1](t) - robot.trajectory[1](t_1)
            dy = robot.trajectory[2](t) - robot.trajectory[2](t_1)
            path_len += (dx**2 + dy**2)**0.5
    print "Total path length", path_len

def main():
    time = Time(0.01)
    colors = 'bgrcmyk'

    pltr = Plotter()

    robot_1 = Robot(1, [2, 2, 0], [8, 6, 0], colors[0])
    robot_2 = Robot(2, [8, 2, 0], [2, 6, 0], colors[1])
    # robot_1 = Robot(1, [5, 8, 0], [4, 4, 0], colors[0])
    # robot_2 = Robot(2, [1, 6, 0], [7, 6, 0], colors[1])

    my_world = World([robot_1, robot_2], time)

    plan(my_world, pltr)
    execute_with_collision(my_world)
    # execute_wo_collision(my_world)

if __name__ == "__main__":
    main()
