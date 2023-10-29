# NOTE: No need to edit this file

import numpy as np

class Track:

    # our tracks are composed of a sample of NUM_POINTS points along a parametric function
    NUM_POINTS = 1000

    # x, y are functions which map t to a point
    # our track is then composed of points (x(t), y(t)) for t in domain t_beg, t_end
    def __init__(self, x, y, t_beg, t_end):
        self.points = []

        dt = (t_end - t_beg) / self.NUM_POINTS
        for i in range(self.NUM_POINTS):
            t_cur = t_beg + dt * i
            self.points.append([x(t_cur), y(t_cur)])
        
        self.points = np.array(self.points)