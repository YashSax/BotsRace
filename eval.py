# NOTE you can use this file instead of attorney to test your code locally
# No need to change this file
# just run server.py and then eval.py

import requests

from bots_race.environment_factory import EnvironmentFactory

class BotsRaceEvaluator:
    def __init__(self):
        pass

    def evaluate(self, base_url):
        track_url = f"{base_url}/bots_race_track"
        observation_url = f"{base_url}/bots_race_observation"

        headers = {
            "Content-Type": "application/json"
        }

        response = requests.request("POST", track_url, json={}, headers=headers)

        env_factory = EnvironmentFactory(debug=False)

        fitness = []
        for i in range(len(env_factory.TRACK_OPTIONS)):
            env = env_factory.get_environment(i)
            obs = env.reset()
            response = requests.request("POST", track_url, json={}, headers=headers)
            done = False
            fitness.append(0)
            while not done:
                response = requests.request("POST", observation_url, json={'observation': obs, 'fitness': fitness}, headers=headers)
                action = response.json()['action']
                obs, fitness[i], done = env.step(action)

        return sum(fitness) / len(fitness)


if __name__ == '__main__':
    evaluator = BotsRaceEvaluator()
    fitness = evaluator.evaluate('http://127.0.0.1:65432')
    print('fitness:', fitness)