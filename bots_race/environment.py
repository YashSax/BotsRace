# NOTE: No need to edit this file

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

import bots_race.utils as utils

class Environment:

    MAX_STEPS = 5000 # how many time steps are allowed to reach the goal
    OFF_TRACK_LIMIT = 50 # how many turns can the robot not be on the track

    def __init__(self, robot, track, debug):
        self.robot = robot
        self.track = track

        self.reset()

        self.debug = debug
        if self.debug: # prepare visualizations
            self.robot_visualization = patches.Circle((self.robot.x, self.robot.y), self.robot.RADIUS, fill=False, color='blue')
            orientation_x = self.robot.x + self.robot.RADIUS * math.cos(self.robot.orientation)
            orientation_y = self.robot.y + self.robot.RADIUS * math.sin(self.robot.orientation)
            self.orientation_visualization, = plt.plot(orientation_x, orientation_y, 'ro', markersize=2)
            plt.gca().add_patch(self.robot_visualization) # add robot as a circle

            # need a comma to destructure return value
            track_x, track_y = zip(*self.track.points)
            self.track_visualization, = plt.plot(track_x, track_y, color='green')
            plt.plot(track_x, track_y)

            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid(True)
            plt.ion()
            plt.show()

    # returns observation, reward, done
    def step(self, action):
        self.num_steps += 1
        time_fitness = 1 - self.num_steps / self.MAX_STEPS
        done = self.num_steps > self.MAX_STEPS

        self.robot.set_accelerations(action[0], action[1])
        self.robot.move()
        self.draw()

        robot_position = np.array([self.robot.x, self.robot.y])
        indices = utils.points_in_range(self.track.points, robot_position, self.robot.RADIUS)
        self.discovered[indices] = 1
        if self.discovered[-1] == 1: # if we have reached the end of the track
            done = True
        point_fitness = sum(self.discovered) / self.track.NUM_POINTS

        # check if the robot is off the track
        if len(indices) == 0:
            self.off_track_count += 1
            if self.off_track_count > self.OFF_TRACK_LIMIT:
                done = True
                time_fitness = 0
        else:
            self.off_track_count = 0

        self.robot.read_sensors(self.track.points)

        observation = [self.robot.orientation / (2 * math.pi)] + self.robot.readings
        return observation, (time_fitness + point_fitness) / 2, done

    def reset(self):
        # tracks if a point has been discovered, ordered by index of point
        self.discovered = np.array([0] * self.track.NUM_POINTS)
        self.num_steps = 0
        self.off_track_count = 0

        observation = [self.robot.orientation / (2 * math.pi)] + self.robot.readings
        return observation

    # for visualization, helps for debug
    def draw(self):
        if not self.debug:
            return

        self.robot_visualization.set_center((self.robot.x, self.robot.y))

        orientation_x = self.robot.x + self.robot.RADIUS * math.cos(self.robot.orientation)
        orientation_y = self.robot.y + self.robot.RADIUS * math.sin(self.robot.orientation)
        self.orientation_visualization.set(xdata=orientation_x, ydata=orientation_y)

        plt.pause(1e-10)