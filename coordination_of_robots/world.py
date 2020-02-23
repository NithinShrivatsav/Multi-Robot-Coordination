import numpy as np
import itertools
import heapq

from trajectory import Trajectory
from rrt import RRT
from astar import Astar
from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
    COLLISION_MARGIN,
    MAX_VEL,
)

class World:

    def __init__(self, robots, time, obstacles=[]):
        self.robots = robots
        self.time = time
        self.x_range = [0, X]
        self.y_range = [0, Y]
        self.obstacles = obstacles
        self.collision = []
        # self.pltr = Plotter()

    def generate_paths(self, algo="a-star"):
        if algo == "a-star":
            for robot in self.robots:
                astar = Astar(robot.pose, robot.goal, self.obstacles)
                robot.path = astar.generate_path()
        elif algo == "rrt":
            for robot in self.robots:
                rrt = RRT(robot.pose, robot.goal, self.obstacles)
                robot.path = rrt.generate_path()
        else:
            print "No such algo currently exists."
            return


    def detect_collision(self):
        for r1, r2 in itertools.combinations(self.robots, 2):
            time = min(r1.trajectory.total_time, r2.trajectory.total_time)
            for i in range(1000):
                t = i*time/1000
                x1, y1 = r1.trajectory.location(t)
                x2, y2 = r2.trajectory.location(t)
                if ((x1 - x2)**2 + (y1 - y2)**2)**0.5 < COLLISION_MARGIN:
                    self.collision.append([(r1.id, r2.id), (x1, y1), t])
                    r1.collision_points.append((x1, y1))
                    r1.collision_impending.append((x1, y1))
                    r1.collision_with.append(r2)
                    r2.collision_points.append((x2, y2))
                    r2.collision_impending.append((x2, y2))
                    r2.collision_with.append(r1)
                    if r1.id < r2.id:
                        r1.collision_priority.append('inc')
                        r2.collision_priority.append('dec')
                    else:
                        r1.collision_priority.append('dec')
                        r2.collision_priority.append('inc')
                    break
        # print self.collision

    def avoid_collision(self):
        for r1, r2 in itertools.combinations(self.robots, 2):
            time = min(r1.trajectory.total_time, r2.trajectory.total_time)
            for i in range(1000):
                t = i*time/1000
                x1, y1 = r1.trajectory.location(t)
                x2, y2 = r2.trajectory.location(t)
                if ((x1 - x2)**2 + (y1 - y2)**2)**0.5 < COLLISION_MARGIN:
                    r1.trajectory.slow_trajectory(t - 3*COLLISION_MARGIN/MAX_VEL)
                    # print "R1", r1.id, r1.trajectory.check_continuity()
                    r1.trajectory.fast_trajectory(t + 0*COLLISION_MARGIN/MAX_VEL)
                    # print "R1", r1.id, r1.trajectory.check_continuity()
                    r2.trajectory.fast_trajectory(t - 3*COLLISION_MARGIN/MAX_VEL)
                    # print "R2", r2.id, r2.trajectory.check_continuity()
                    r2.trajectory.slow_trajectory(t + 0*COLLISION_MARGIN/MAX_VEL)
                    # print "R2", r2.id, r2.trajectory.check_continuity()
                    # print "-----------------"
        if self.detect_collision():
            self.avoid_collision()
