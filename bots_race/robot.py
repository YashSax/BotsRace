# NOTE: No need to edit this file

import math
import numpy as np

import bots_race.utils as utils

class Robot:

    # size of the robot
    RADIUS = .04
    # sensors are just radially arranged from center of robot starting with 
    # right in front of the center (in direction of robot orientation)
    NUM_SENSORS = 4
    SENSOR_RADIUS = .02
    SENSOR_OFFSET = .01 # how far away from center of robot is each sensor

    MAX_LINEAR_VELOCITY_MAGNITUDE = .005
    MAX_ANGULAR_VELOCITY_MAGNITUDE = .1
    MAX_LINEAR_ACCELERATION_MAGNITUDE = .001
    MAX_ANGULAR_ACCELERATION_MAGNITUDE = .02

    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation # in radians
        self.readings = self.NUM_SENSORS * [0]
        self.linear_velocity = 0 # velocity in direction of orientation
        self.angular_velocity = 0 # how much the orientation is changing

        # these will be the dials used to actually control the robot
        self.linear_acceleration = 0
        self.angular_acceleration = 0

    # a list of size NUM_SENSORS where each number is between 0 and 1, where 0 means there are 
    # no points from the track in the sensor's radius, and 1 means every single point
    # of the track is in the sensor's radius (1 should never happen)
    def read_sensors(self, track_points):
        dtheta = 2 * math.pi / self.NUM_SENSORS
        for i in range(self.NUM_SENSORS):
            sensor_position = [self.x + self.SENSOR_OFFSET * math.cos(dtheta * i + self.orientation),
                               self.y + self.SENSOR_OFFSET * math.sin(dtheta * i + self.orientation)]
            sensor_position = np.array(sensor_position)
            num_points_in_range = len(utils.points_in_range(track_points, sensor_position, self.SENSOR_RADIUS))
            self.readings[i] = num_points_in_range / len(track_points)
        return self.readings

    # moves the robot based on current state
    def move(self):
        self.x += self.linear_velocity * math.cos(self.orientation)
        self.y += self.linear_velocity * math.sin(self.orientation)
        self.orientation += self.angular_velocity

        self.linear_velocity += self.linear_acceleration
        self.angular_velocity += self.angular_acceleration
        self.linear_velocity = utils.bound(self.linear_velocity,
                                           -self.MAX_LINEAR_VELOCITY_MAGNITUDE,
                                           self.MAX_LINEAR_VELOCITY_MAGNITUDE)
        self.angular_velocity = utils.bound(self.angular_velocity,
                                            -self.MAX_ANGULAR_VELOCITY_MAGNITUDE,
                                            self.MAX_ANGULAR_VELOCITY_MAGNITUDE)
        # print("Ang vel:", self.angular_velocity, "Lin vel:", self.linear_velocity)

    def set_accelerations(self, linear, angular):
        self.linear_acceleration = utils.bound(linear,
                                               -self.MAX_LINEAR_ACCELERATION_MAGNITUDE,
                                               self.MAX_LINEAR_ACCELERATION_MAGNITUDE)
        self.angular_acceleration = utils.bound(angular,
                                                -self.MAX_ANGULAR_ACCELERATION_MAGNITUDE,
                                                self.MAX_ANGULAR_ACCELERATION_MAGNITUDE)