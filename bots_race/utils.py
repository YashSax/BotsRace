# NOTE: No need to edit this file

import numpy as np

# returns indices of points which are within r distance of point p
def points_in_range(points, p, r):
    distances = np.linalg.norm(points - p, axis=1)
    indices = np.where(distances <= r)[0]
    return indices

# ensures a return val between lo and hi
def bound(val, lo, hi):
    return max(min(val, hi), lo)