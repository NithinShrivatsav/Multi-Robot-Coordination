import heapq
import math

from plotter import Plotter
from constants import(
    X, Y,
    X_GRIDS, Y_GRIDS,
    COLLISION_MARGIN,
    DEBUG,
)

class Astar:

    def __init__(self, start, goal, obstacles):

        grid_size_x = X/X_GRIDS; grid_size_y = Y/Y_GRIDS
        self.start = (math.floor(start[0]/grid_size_x), math.floor(start[1]/grid_size_y))
        self.goal = (math.floor(goal[0]/grid_size_x), math.floor(goal[1]/grid_size_y))
        self.obstacles = []
        if obstacles:
            self.square_obs(*obstacles)

        self.directions = [(-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]

        self.path = [];              self.open_list = []
        self.cost = {};              self.came_from = {}
        heapq.heappush(self.open_list, (self.heuristic(self.start), self.start))
        self.came_from[self.start] = None
        self.cost[self.start] = 0

        if DEBUG:
            self.pltr = Plotter()

    def square_obs(self, l, w, start):
        l_grids = int(math.floor((l+2*COLLISION_MARGIN)*X_GRIDS/X))
        w_grids = int(math.floor((w+2*COLLISION_MARGIN)*Y_GRIDS/Y))
        start_x = int(math.floor((start[0]-COLLISION_MARGIN)*X_GRIDS/X))
        start_y = int(math.floor((start[1]-COLLISION_MARGIN)*Y_GRIDS/Y))
        for i in range(l_grids):
            for j in range(w_grids):
                self.obstacles.append((start_x+i, start_y+j))

    def heuristic(self, location):
        return abs((location[0]-self.goal[0])**2+(location[1]-self.goal[1])**2)**0.5

    def is_obstacle(self, location):
        return location in self.obstacles

    def generate_path(self):
        while True:
            current = heapq.heappop(self.open_list)[1]
            if(current[0] == self.goal[0] and current[1] == self.goal[1]):
                while current in self.came_from:
                    self.path.append([current[0]*X/X_GRIDS, current[1]*Y/Y_GRIDS])
                    current = self.came_from[current]
                self.path.reverse()
                return self.path
            else:
                for i,j in self.directions:
                    neighbor = (current[0]+i,current[1]+j)
                    if neighbor[0] < 0 or neighbor[0] >= X_GRIDS or neighbor[1] < 0 or neighbor[1] >= Y_GRIDS:
                        continue
                    if self.is_obstacle(neighbor):
                        continue
                    if neighbor not in self.cost or self.cost.get(neighbor,0) > self.cost[current]+1:
                        self.came_from[neighbor] = current
                        self.cost[neighbor] = self.cost[current]+1
                        priority = self.cost[neighbor] + self.heuristic(neighbor)
                        heapq.heappush(self.open_list,(priority,neighbor))

    def plot(self):
        start = self.start[0]*X/X_GRIDS, self.start[1]*Y/Y_GRIDS
        goal = self.goal[0]*X/X_GRIDS, self.goal[1]*Y/Y_GRIDS
        self.pltr.draw_start_end(start, goal, 'k')
        self.pltr.sleep(0.01)
        for obs in self.obstacles:
            self.pltr.plotting(obs[0]*X/X_GRIDS, obs[1]*Y/Y_GRIDS, 'r')
        self.pltr.sleep(0.01)
        for p in self.path:
            self.pltr.plotting(p[0], p[1], 'g')
            self.pltr.sleep(0.01)
        self.pltr.sleep(2)

if __name__ == "__main__":
    astar = Astar((1,1), (8,8), [2, 2, (4,4)])
    print astar.generate_path()
    astar.plot()
