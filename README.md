# Biped Robotics
Projects for biped and servo robotics, specifically evolving bipeds to perform a task.

Bipedal walking is a complex task to perform, there are all sorts of problems such as non-linearities within motor control and changes within environmental conditions. This repository implemented the use of adaptive methods on physical robots, in order to make biped robot projects simpler.

Genetic algorithms (GA) allow systems to adapt randomly to find solutions. This minimizes the time that the developer needs to spend creating a solution. Some tasks are too complicated or time consuming for a developer to code a solution. Tasks such as walking require balance and correct movement all in concurrence with one another. Using a GA is a known approach to satisfying a solution while considering both constraints. An issue arises where these
solutions have an enormous search space and can be impractical for solving a solution quickly . Research on bipedal genetic algorithms typically takes more
generations to arrive at a candidate solution the more motors there are. 
This is an interesting topic due to the significant part physical robots will one day have in society. We think very little about the complex movements we perform everyday. Being able to master optimization for robotics will help get to this stage. In this experiment we will see how key to the performance of an
algorithm mutation rate is.

# GA walking
Using a Genetic algorithm we are able to encode motor positons in a series of steps. The genotype could encode information as such:
```
genotype=[[0,0,0,0],[1,0,0,-1]]
```
Where a 1 moves a motor forward by x-degrees, and a -1 moves it back by x-degrees. Each index in the outer array represents steps , and the inside arrays represent each motor. [step1, step2, step3] and [motor1, motor2, motor3, motor4]. The previous example would not move in the first step, and then move motor 1 and motor 4 in opposite directions. This could also be set explicitly as randomized degrees movement: 
```
genotype=[[0,0,0,0],[10,30,0,-1]]
```

# GA optimization for walking


# Essential Reading
https://towardsdatascience.com/evolving-a-robot-to-walk-using-python-83417ca3df2a



