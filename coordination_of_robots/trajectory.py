import numpy as np
from scipy.optimize import minimize_scalar
from plotter import Plotter

from constants import(
    DEFAULT_TIME,
    MAX_VEL,
)

class Trajectory:

    def __init__(self, pts, order):
        self.order = order
        self.total_time = 0
        self.generate_trajectory(pts)

    def __str__(self):
        string = "Number of segments: " + str(len(self.trajectory)) + '\n'
        for piece in self.trajectory:
            string += "Duration: " + str(piece[0]) + '\n'
            string += "X: \n" + str(piece[1]) + '\n'
            string += "Y: \n" + str(piece[2]) + '\n'
            string += "\n"
        return string

    def generate_trajectory(self, pts):
        x = [p[0] for p in pts]
        y = [p[1] for p in pts]
        t = np.linspace(0, DEFAULT_TIME, len(pts))
        p = np.polyfit(t, zip(x, y), self.order)
        poly1d_x = np.poly1d(p[:,0])
        poly1d_y = np.poly1d(p[:,1])

        dx = np.polyder(poly1d_x)
        dy = np.polyder(poly1d_y)
        velocity_sq = dx * dx + dy * dy

        duration = DEFAULT_TIME
        res = minimize_scalar(-1 * velocity_sq, bounds=(0, DEFAULT_TIME), method='bounded')
        max_velocity = velocity_sq(res.x)**0.5

        duration = DEFAULT_TIME * max_velocity/MAX_VEL
        scaling_polynomial = np.poly1d([MAX_VEL/max_velocity, 0])
        poly1d_x = np.polyval(poly1d_x, scaling_polynomial)
        poly1d_y = np.polyval(poly1d_y, scaling_polynomial)
        self.trajectory = [[duration, poly1d_x, poly1d_y]]
        self.total_time = self.calc_total_time()

    def location(self, time):
        duration = 0
        for piece in self.trajectory:
            duration += piece[0]
            if time <= duration:
                t = time - duration + piece[0]
                return piece[1](t), piece[2](t)
        return piece[1](piece[0]), piece[2](piece[0])

    def derivative(self, time):
        duration = 0
        for piece in self.trajectory:
            duration += piece[0]
            if time <= duration:
                t = time - duration + piece[0]
                return np.polyder(piece[1])(t), np.polyder(piece[2])(t)
        return 0, 0

    def double_derivative(self, time):
        duration = 0
        for piece in self.trajectory:
            duration += piece[0]
            if time <= duration:
                t = time - duration + piece[0]
                return np.polyder(np.polyder(piece[1]))(t), np.polyder(np.polyder(piece[2]))(t)
        return 0, 0

    def slow_trajectory(self, time):
        duration = 0
        for piece_id, piece in enumerate(self.trajectory):
            duration += piece[0]
            if time < duration:
                piece[0] = time - duration + piece[0]
                scaling_poly = np.poly1d([0.5, piece[0]])
                new_piece = [2.0*(duration - time), np.polyval(piece[1], scaling_poly), np.polyval(piece[2], scaling_poly)]
                self.trajectory.insert(piece_id+1, new_piece)
                break
        piece_id += 2
        scaling_poly = np.poly1d([0.5, 0])
        while piece_id < len(self.trajectory):
            piece = self.trajectory[piece_id]
            self.trajectory[piece_id] = [piece[0]*2.0, np.polyval(piece[1], scaling_poly), np.polyval(piece[2], scaling_poly)]
            piece_id += 1
        self.total_time = self.calc_total_time()
        
    def fast_trajectory(self, time):
        duration = 0
        for piece_id, piece in enumerate(self.trajectory):
            duration += piece[0]
            if time < duration:
                piece[0] = time - duration + piece[0]
                scaling_poly = np.poly1d([2, piece[0]])
                new_piece = [(duration - time)/2.0, np.polyval(piece[1], scaling_poly), np.polyval(piece[2], scaling_poly)]
                self.trajectory.insert(piece_id+1, new_piece)
                break
        piece_id += 2
        scaling_poly = np.poly1d([2, 0])
        while piece_id < len(self.trajectory):
            piece = self.trajectory[piece_id]
            self.trajectory[piece_id] = [piece[0]/2.0, np.polyval(piece[1], scaling_poly), np.polyval(piece[2], scaling_poly)]
            piece_id += 1
        self.total_time = self.calc_total_time()

    def calc_total_time(self):
        return sum(piece[0] for piece in self.trajectory)

    def check_continuity(self):
        for piece_id, piece in enumerate(self.trajectory):
            if piece_id + 1 < len(self.trajectory):
                next_piece = self.trajectory[piece_id + 1]
                if (piece[1](piece[0]) - next_piece[1](0)) <= 0.01 and (piece[2](piece[0]) - next_piece[2](0)) <= 0.01:
                    continue
                else:
                    print piece_id
                    print piece[1](piece[0]), next_piece[1](0)
                    print piece[2](piece[0]), next_piece[2](0)
                    return False
        return True

# if __name__=="__main__":
#     traj = Trajectory([(1,1), (2,2), (3,3), (4,4)], 3)
#     traj.trajectory = [[4, np.poly1d([1, 2, 3, 4]), np.poly1d([2, 3, 4, 5])]]
#     traj.fast_trajectory(0.5)
#     traj.slow_trajectory(1)
#     traj.fast_trajectory(1.5)
#     traj.slow_trajectory(2)
#     traj.slow_trajectory(2.5)
#     traj.fast_trajectory(3)
#     print traj
#     print traj.check_continuity()
