# NOTE this is how you should structure your submissions
# you shouldn't need to edit this file, just the solution.py file

# this file creates a server which allows our grader to communicate with your code

# go ahead and give it a try to see yourself on the leaderboard!
# run server.py and then attorney

from solution import Solution

from flask import Flask, jsonify, request
app = Flask(__name__)

solution = Solution()

@app.route("/bots_race_track", methods=["POST"])
def bots_race_track():
    solution.track()
    return {"status": "success"}

@app.route("/bots_race_observation", methods=["POST"])
def bots_race_observation():
    data = request.get_json()
    action = solution.get_action(data['observation'])
    return {"action": action}
    
if __name__ == "__main__":
    app.run(host="127.0.0.1", port=65432)