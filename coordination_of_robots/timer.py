import numpy as np

class Time():
    
    def __init__(self, dt):
        self.t = 0
        self.dt = dt

    def time(self):
        return self.t

    def step(self):
        self.t += self.dt

    def sleep(self, duration):
        for t in np.arange(self.t, self.t + duration, self.dt):
            self.step()
