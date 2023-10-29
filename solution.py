import math
from bots_race.environment_factory import EnvironmentFactory
import matplotlib.pyplot as plt
import pickle

class Solution:
    def __init__(self):
        # TODO code to initialize your solution
        self.ANG_ERR_THRESH = 0.01
        self.ANG_PID_P = 0.1
        self.ANG_PID_D = 5
        self.ANG_PID_I = 0
        self.damping = 0.5
        self.prev_err = 0
        self.cum_err = 0

        self.started = False
        self.start_diff = None

        self.prev_done = False
        self.vroomed = False

        self.data = {
            "corrections" : [],
            "ps" : [],
            "is" : [],
            "ds" : [],
            "target_angle" : []
        }

    def track(self):
        # TODO fill in code here which initializes your controller
        # whenever your robot is placed on a new track
        pass
    
    def find_angle(self, robot_observation):
        ca, top, left, bottom, right = robot_observation
        if self.vroomed:
            bottom = 0
        # print(top, left, bottom, right)
        relative_angle = (0.5 + math.atan2(right - left, bottom - top) / (2 * math.pi)) % 1
        # print("Relative angle:", relative_angle)
        return (ca % 1 + relative_angle) % 1
    
    def turn_to_angle(self, angle, robot_observation):
        # Returns ang_acc, done
        curr_angle, _, _, _, _ = robot_observation
        curr_angle %= 1
        error = curr_angle - angle

        if error > 0.5:
            error = -1 * (error - 0.5)
        elif error < -0.5:
            error = -1 * (error + 0.5)
        
        error = -1 * error
        p_component = error
        d_component = error - self.prev_err
        i_component = error + self.cum_err 

        # print("P:", p_component, "I:", i_component, "D:", d_component)
        correction = p_component * self.ANG_PID_P + d_component * self.ANG_PID_D + i_component * self.ANG_PID_I
        self.data["corrections"].append(abs(correction))
        self.data["ps"].append(abs(p_component * self.ANG_PID_P))
        self.data["is"].append(abs(d_component * self.ANG_PID_D))
        self.data["ds"].append(abs(i_component * self.ANG_PID_I))
        self.data["target_angle"].append(angle)

        print("Correction magnitude:", abs(correction))

        self.prev_err = error
        self.cum_err += error

        done = abs(angle - curr_angle) < self.ANG_ERR_THRESH
        return [correction, done]

    def done_rising_edge(self, done):
        if not self.prev_done and done:
            return True
        return False
    
    def done_falling_edge(self, done):
        if self.prev_done and not done:
            return True
        return False

    # should return [linear_acceleration, angular_acceleration]
    def get_action(self, robot_observation):
        angle = self.find_angle(robot_observation)
        if angle is None:
            return [0, 0]
        angle %= 1
        ang_acc, done = self.turn_to_angle(angle, robot_observation)
        if self.start_diff == None:
            self.start_diff = abs(angle - robot_observation[0])
        
        lin_acc = 0
        if self.done_rising_edge(done) and not self.vroomed:
            lin_acc = 0.001
            self.vroomed = True
        elif self.done_falling_edge(done):
            lin_acc = -0.001
        
        # print(f"Lin: {lin_acc}, Ang: {ang_acc}")
        return [lin_acc, ang_acc]



"""
Find the angle
Go to the angle (no linear acceleration)

linear acceleration turns on

repeat:
find the angle (no bottom)
go to the angle
"""

# this is example of code to test your solution locally
if __name__ == '__main__':
    solution = Solution()

    # TODO check out the environment_factory.py file to create your own test tracks
    env_factory = EnvironmentFactory(debug=True)
    env = env_factory.get_random_environment()

    done = False
    fitness = 0
    robot_observation = env.reset()

    # deviation = 0
    # num_steps = 0
    # start_at = 50
    # stop_at = 1200
    while not done:
        robot_action = solution.get_action(robot_observation)
        robot_observation, fitness, done = env.step(robot_action)
        # if num_steps > stop_at:
        #     break
        # num_steps += 1
    
    # print("Average deviation:", deviation / (num_steps - start_at))
    print("Starting diff:", solution.start_diff)
    print('Solution score:', fitness)
    
    with open("./solution_out", "wb") as f:
        pickle.dump(solution.data, f)