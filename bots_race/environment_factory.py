import random
import math

from bots_race.environment import Environment
from bots_race.track import Track
from bots_race.robot import Robot

class EnvironmentFactory:

    # tracks are parametric functions with a domain of t between 0 and 1
    T_BEG = 0
    T_END = 1

    # TODO hey Datathoners!
    # be sure to test out your own custom tracks
    TRACK_OPTIONS = [[lambda t: t, lambda t: t],
                     [lambda t: 2 * (t - .5), lambda t: t**2],
                     [lambda t: math.cos(2 * t), lambda t: math.sin(2 * t)],
                     [lambda t: -(t + .2) ** 3, lambda t: 2 * t ** 2 - 1],
                     [lambda t: math.e ** (2 * t) / (math.e ** t + 1) - 1, lambda t: 2 * math.log(t + .01) / (math.log(t + .01) - 1) - 1],
                     [lambda t: -2 * t + 1, lambda t: -2 * t + 1]]

    def __init__(self, debug):
        self.debug = debug

    # gets a random environment based on TRACK_OPTIONS, starts robot at x(0), y(0) at random orientation
    def get_random_environment(self, idx):
        track = self.create_track(*self.TRACK_OPTIONS[idx])#*random.choice(self.TRACK_OPTIONS)) # 1 IS THE BAD ONE
        robot = Robot(track.points[0][0], track.points[0][1], random.random() * 2 * math.pi)
        return Environment(robot, track, self.debug)

    # gets the environment at index i, starts robot at x(0), y(0) with orientation of 0 radians
    def get_environment(self, track_index):
        track = self.create_track(*self.TRACK_OPTIONS[track_index])
        robot = Robot(track.points[0][0], track.points[0][1], 0)
        return Environment(robot, track, self.debug)

    def create_track(self, track_x, track_y):
        return Track(track_x, track_y, self.T_BEG, self.T_END)
