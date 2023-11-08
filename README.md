## Problem Statement
Simply put: Make a robot follow a given path by passing in values for linear and angular acceleration. Full information [here](https://tamudatathon.com/challenges/docs/BotsRace):

## Demo
https://github.com/YashSax/BotsRace/assets/46911428/62933265-ed1a-4761-a767-a8805ef3424b

## Inspiration
We initially tried to throw Reinforcement Learning at the problem by using [NEAT](https://en.wikipedia.org/wiki/Neuroevolution_of_augmenting_topologies) (NeuroEvolution of Augmenting Topologies) and Genetic Algorithms to generate many potential candidate drivers, evaluate their fitness, remove the worst, and combine the best. However, after about half an hour of training and much more time invested in playing with various hyperparameters, we hit a dead end. At that point, we decided to instead approach the problem from a first-principles approach using [PID control](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller). In our implementation journey, we ran into a lot of interesting problems!

## What it does
Our program solves the Bots Race Challenge, which involves making a bot, equipped with four simple sensors, follow a given track. While the challenge sounds simple, the robot is extremely fickle and we found through trial and (much) error that it's incredibly important to regulate angular acceleration to a point where the bot loses control. 

## Challenges we ran into and our Solutions
The first challenge we ran into was to try and figure out how to structure our program. At its very core, our program does two things in one iteration: 1) Figure out what angle to turn to. 2) Go to that angle. While the two tasks sound simple, there's a lot of nuance to each one. Here are some of the challenges we ran into and how we solved them:
 - Challenge: Given just four numbers describing the proportion of points in each sensor's radius, how can you come up with an accurate measure on where to turn?
    - Solution: Consider each sensor to be a vector pointing in its direction with a magnitude of its value. Then, add all these vectors together to create the direction where points are most concentrated. Note that when the bot is fully over the line, we'll need to ignore the bottom sensor so it doesn't counteract the top sensor
 - Challenge: How can we turn towards a specific angle when the only thing we can control is the angular acceleration (not even the angular velocity!!)?
    - Solution: For this, we use PID (Proportional, Integral, Derivative) Control. We define the error of a given state to be the difference between the current angle and the desired angle. Our update (new angular acceleration) is then a function of the error, the derivative of the error, and the integral of the error. From there, we can then change the coefficients to fine-tune. 

## Accomplishments that we're proud of
I think the most interesting thing we did was use a smoothing technique inspired by Neural Network optimizers such as RMSProp and Adam to smooth noisy estimates for the desired angle. More information can be found on this [wiki](https://en.wikipedia.org/wiki/Stochastic_gradient_descent#RMSProp)

