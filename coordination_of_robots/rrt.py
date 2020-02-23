import numpy as np
import math
import random
from plotter import Plotter

from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
    COLLISION_MARGIN,
    DEBUG,
)

class RRT_node:
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent

class RRT:
    def __init__(self, start, goal, obstacles, min_dist = 0.2, thresh = 0.2, max_nodes = 100000):
        self.start = RRT_node(start, None)
        self.goal = RRT_node(goal, None)
        self.obstacles = []
        if obstacles:
            self.square_obs(*obstacles)

        self.min_distance = min_dist
        self.threshold = thresh
        self.max_nodes = max_nodes
        self.nodes = [self.start]
        self.path = []
        random.seed(52)
        if DEBUG:
            self.pltr = Plotter()
            self.pltr.draw_start_end(start, goal, 'k')
            for obs in self.obstacles:
                self.pltr.plotting(obs[0], obs[1], 'r')
            self.pltr.sleep(0.01)

    def square_obs(self, l, w, start):
        l_num = int(math.floor(l/COLLISION_MARGIN))
        w_num = int(math.floor(w/COLLISION_MARGIN))
        for i in range(l_num+2):
            for j in range(w_num+2):
                self.obstacles.append((start[0]+(i-1)*COLLISION_MARGIN, start[1]+(j-1)*COLLISION_MARGIN))

    def distance(self, pt1, pt2):
        return ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**0.5

    def movement(self, pt1, pt2):
        if self.distance(pt1, pt2) < self.min_distance:
            return pt2
        else:
            theta = np.arctan2(pt2[1] - pt1[1], pt2[0] - pt1[0])
            return pt1[0]+self.min_distance*math.cos(theta), pt1[1]+self.min_distance*math.sin(theta)

    def is_collision(self, point):
        for obs in self.obstacles:
            if self.distance(point, obs) < 0.5:
                return True
        return False

    def closest_neighbor(self, pt):
        distances_list = [self.distance(pt, n.point) for n in self.nodes]
        return distances_list.index(min(distances_list))

    def get_random_point(self):
        while True:
            pt = random.random()*X, random.random()*Y
            if not self.is_collision(pt):
                return pt

    def generate_path(self):
        count = 0
        while True:
            count += 1
            if DEBUG:
                if count %100 == 0:
                    print("RRT iteration: ",count)
            if count < self.max_nodes:
                while True:
                    random_pt = self.get_random_point()
                    parent = self.nodes[self.closest_neighbor(random_pt)]
                    next_point = self.movement(parent.point, random_pt)
                    if not self.is_collision(next_point):
                        self.nodes.append(RRT_node(next_point, parent))
                        break
                if DEBUG:
                    self.pltr.draw_line(parent.point, next_point, 'k', 1)
                    self.pltr.sleep(0.001)
                if self.distance(next_point, self.goal.point) < self.threshold:
                    self.goal.parent = self.nodes[-1]
                    break
            else:
                #No path
                print("No path found.")
                return -1

        if DEBUG:
            self.pltr.sleep(1)
        curr_node = self.goal
        while True:
            self.path.append(curr_node.point)
            if curr_node.parent == None:
                break
            if DEBUG:
                self.pltr.draw_line(curr_node.point, curr_node.parent.point, 'g', 2.5)
            curr_node = curr_node.parent
        self.path = self.path[::-1]
        if DEBUG:
            self.pltr.sleep(2)
        return self.path

if __name__=="__main__":
    rrt = RRT((1,1), (8, 8), [2, 2, (4,4)])
    rrt.generate_path()
    print(rrt.path)
