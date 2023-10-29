import math
from bots_race.environment_factory import EnvironmentFactory
import matplotlib.pyplot as plt
import pickle
import random

class Solution:
    def __init__(self):
        # TODO code to initialize your solution
        self.ANG_ERR_THRESH = 0.01
        self.ANG_PID_P = 0.1
        self.ANG_PID_D = 5
        self.ANG_PID_I = 0
        self.damping = 0.25
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
            "smoothed_angles" : [],
            "unsmoothed_angles" : [],
            "robot_angles" : [],
            "tops" : [],
            "lefts" : [],
            "bottoms" : [],
            "rights" : [],
            "vroom_idx" : 0
        }

        self.beta = 0.95
        self.past_angles = []
        self.counter_since_vroom = 0
        self.max_depth = 100
        self.i = 0
    
    def track(self):
        # TODO fill in code here which initializes your controller
        # whenever your robot is placed on a new track
        pass
    
    def decompose(self, ang):
        rad = 2 * math.pi * ang
        x, y = math.cos(rad), math.sin(rad)
        return x, y

    def compose(self, x, y):
        return math.atan2(y, x) * 1 / (2 * math.pi)

    def get_smoothed(self, a_list, beta=0.95):
        arr = [self.decompose(a) for a in a_list]
        weights = [(1 - beta) * beta ** i for i in range(len(arr))][::-1]
        x_sum, y_sum = sum([a[0] * w for a, w in zip(arr, weights)]), sum([a[1] * w for a, w in zip(arr, weights)])
        return self.compose(x_sum, y_sum)

    # def get_smoothed(self, idx, val_arr, curr_depth, beta=0.9):
    #     if idx == 0 or curr_depth == 0:
    #         return val_arr[idx]
    #     return val_arr[idx] * (1 - beta) + self.get_smoothed(idx - 1, val_arr, curr_depth - 1)  * beta

    def find_angle(self, robot_observation, smooth=False, append_to_list=False):
        angle = self.find_angle_helper(robot_observation) % 1
        self.data["unsmoothed_angles"].append(angle)
        self.data["robot_angles"].append(robot_observation[0] % 1)
        if not append_to_list:
            self.data["smoothed_angles"].append(angle)
            return angle
        self.past_angles.append(angle)
        if not smooth:
            self.data["smoothed_angles"].append(angle)
            return angle
        smoothed_angle = self.get_smoothed(self.past_angles)
        self.data["smoothed_angles"].append(smoothed_angle) 
        return smoothed_angle

    def find_angle_helper(self, robot_observation):
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

        correction = p_component * self.ANG_PID_P + d_component * self.ANG_PID_D + i_component * self.ANG_PID_I
        self.data["corrections"].append(abs(correction))
        self.data["ps"].append(abs(p_component * self.ANG_PID_P))
        self.data["is"].append(abs(d_component * self.ANG_PID_D))
        self.data["ds"].append(abs(i_component * self.ANG_PID_I))

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
        self.i += 1
        self.counter_since_vroom += self.vroomed
        angle = self.find_angle(robot_observation, append_to_list=self.vroomed, smooth=self.counter_since_vroom >= 40)
        
        ca, top, left, bottom, right = robot_observation
        self.data["tops"].append(top)
        self.data["lefts"].append(left)
        self.data["bottoms"].append(bottom)
        self.data["rights"].append(right)
        # if self.vroomed:
        #     input()

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
            self.data["vroom_time"] = self.i
        elif self.done_falling_edge(done):
            lin_acc = -0.001
        
        return [lin_acc, ang_acc]

def run_tests():
    num_runs_per_track = 10
    overall = 0
    for idx in range(6):
        fit_sum = 0
        for _ in range(num_runs_per_track):
            fit_sum += run_test(idx)
        print(f"Average fitness for test {idx} = {fit_sum / num_runs_per_track}")
        overall += fit_sum / num_runs_per_track
    print(f"Overall average = {overall / 6}")

def run_test(idx):
    solution = Solution()

    # TODO check out the environment_factory.py file to create your own test tracks
    env_factory = EnvironmentFactory(debug=False)
    env = env_factory.get_random_environment(idx)

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

    return fitness

# this is example of code to test your solution locally
if __name__ == '__main__':
    run_tests()
    assert False
    solution = Solution()

    # TODO check out the environment_factory.py file to create your own test tracks
    env_factory = EnvironmentFactory(debug=True)
    env = env_factory.get_random_environment(5)

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
        # input()
        # if num_steps > stop_at:
        #     break
        # num_steps += 1
    
    # print("Average deviation:", deviation / (num_steps - start_at))
    print("Starting diff:", solution.start_diff)
    print('Solution score:', fitness)
    
    with open("./solution_out", "wb") as f:
        pickle.dump(solution.data, f)
