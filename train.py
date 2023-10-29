'''
TODO: Deal with intersecting paths, when looking to increment currWaypoint, only look for the next
or previous one, not just the one you're in range of

'''

import numpy as np
import pickle
import neat
from bots_race.environment_factory import EnvironmentFactory

generations = 20
runs_per_net = 5

def eval_driver(net):
    env_factory = EnvironmentFactory(debug=False)
    env = env_factory.get_random_environment()

    done = False
    fitness = 0
    robot_observation = env.reset()

    curr_pos, top_points, right_points, bottom_points, left_points = robot_observation
    while ((top_points <= right_points) or (top_points <= left_points) or (top_points <= bottom_points)):
        robot_observation, fitness, done = env.step([0, 0.0005])
        curr_pos, top_points, right_points, bottom_points, left_points = robot_observation

    while not done:
        robot_action = net.activate(np.array(robot_observation))
        robot_observation, fitness, done = env.step(robot_action)
    return fitness

def eval_genome(genome, config):
    # create the network based off the config file
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    fitnesses = []

    reward = 0
    for runs in range(runs_per_net):
        reward = eval_driver(net)
        fitnesses.append(reward)
    return sum(fitnesses) / len(fitnesses) * 100

def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
        genome.fitness = eval_genome(genome, config)

def main():
    config_path = "./config-feedforward"
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
    pop = neat.Population(config)
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)
    pop.add_reporter(neat.StdOutReporter(True))
    
    NUM_CPU_CORES = 4
    pe = neat.ParallelEvaluator(NUM_CPU_CORES, eval_genome)
    winner = pop.run(pe.evaluate,generations)

    net = neat.nn.FeedForwardNetwork.create(winner, config)
    env_factory = EnvironmentFactory(debug=True)
    env = env_factory.get_random_environment()

    done = False
    fitness = 0
    robot_observation = env.reset()
    curr_pos, top_points, right_points, bottom_points, left_points = robot_observation
    while ((top_points <= right_points) or (top_points <= left_points) or (top_points <= bottom_points)):
        robot_observation, fitness, done = env.step([0, 0.0005])
        curr_pos, top_points, right_points, bottom_points, left_points = robot_observation
    while not done:
        robot_action = net.activate(np.array(robot_observation))
        robot_observation, fitness, done = env.step(robot_action)

    # Save the winner.
    with open('./trained_driver', 'wb') as f:
        pickle.dump(winner, f)

    # Show winning neural network
    print(winner)

if __name__ == "__main__":
    main()