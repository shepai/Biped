from adafruit_servokit import ServoKit
from board import D9, D6
from random import randint, choice, random
from time import sleep
from adafruit_hcsr04 import HCSR04

sonar = HCSR04(trigger_pin=D9, echo_pin=D6)
kit = ServoKit(channels=8)

#setup motors and genotype
"""
PopGenotype=[]
popsize=3
for i in range(popsize): #population size 10
    PopGenotype.append([[choice([0,30,-30,0,0]) for x in range(4)] for i in range(6)])#n steps as max
"""

def getDist(): #get the distane reading
    dist=None
    while dist==None: #get numeric value
        try:
            dist=sonar.distance
        except RuntimeError: #do not return error
            pass
    return int(dist)
def move(angles):
    for i,ang in enumerate(angles): #move the servos by the given angles
        if kit.servo[i].angle+ang>=0 and kit.servo[i].angle+ang<=180: #validate the change is in range
            kit.servo[i].angle+=ang #set each servo to change
            sleep(0.2) #prevent over current draw
    sleep(1) #give time to rest

def st(): #move all servors to the given angles
    kit.servo[0].angle=90 #set servo angle
    kit.servo[1].angle=100 #set servo angle
    kit.servo[2].angle=140 #set servo angle
    kit.servo[3].angle=110 #set servo angle
def mutate(geno,rate=0.2): #mutate the genotype with the given rate
    for i in range(len(geno)):
        if random() < rate: #if rate% chance
            pos=geno[i]
            n2=randint(0,len(pos)-1) #mutate again
            c=[0,30,-30,0,0]
            c.remove(pos[n2])
            pos[n2]=choice(c) #unique
            geno[i]=pos.copy()
    return geno #return mutated

for z in range(10): #loop through the sample size of rates (10%, 20%,..., N%)
    lastFitness=0
    PopGenotype=[[[-30, 0, 30, 0], [0, 0, -30, 0], [0, 0, 0, 0], [30, 0, 30, 30], [30, 0, 30, 0], [0, 0, -30, 0]], [[30, 0, 0, 0], [0, 30, 0, -30], [0, 0, 0, 0], [0, 0, 30, 0], [30, 0, -30, -30], [30, 0, 0, -30]], [[0, 0, 0, 0], [0, 0, 30, 0], [0, 0, 30, 0], [0, 30, 0, -30], [0, 0, -30, 0], [0, 0, 0, -30], [0, 0, 30, 0], [0, 30, 0, -30], [0, 0, -30, 0], [0, 30, 0, -30]]]

    for i in range(15): #total of 15 generations
            print("Generation",i+1)
            st()
            sleep(2) #time to reset if fallen over
            distNo1=getDist() #start dist

            n1=randint(0,popsize-1) #gather the population samples
            n2=randint(0,popsize-1)

            current=PopGenotype[n1].copy()
            current=mutate(current,rate=(z+1)/10)

            for step in current: #move motors by each position specified in genotype
                move(step)

            cd1=getDist() #get the new distance

            current=PopGenotype[n2].copy()
            current=mutate(current,rate=(z+1)/10)
            st() #reset to start position to fairly test the next generation
            sleep(2) #time to reset if fallen over
            distNo2=getDist() #start dist

            for step in current: #move motors by each position specified in genotype
                move(step)
            cd2=getDist() #get the new distance

            print(max(distNo1-cd1,distNo2-cd2))

            if distNo1-cd1>distNo2-cd2: #tournament selection
                PopGenotype[n2]=PopGenotype[n1].copy()
            else:
                PopGenotype[n1]=PopGenotype[n2].copy()

    print((z+1)*10) #show console what is going on
