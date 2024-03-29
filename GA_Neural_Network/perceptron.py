
"""
GA will evolve a neural nework agent controlling a robotic Chassis to walk
Overview
a series of motor instructions encoded as m1,m2,m3,m4 with the current gyroscopic position
and this will help predict the next step
7 inputs, 4 outputs for a motor system of 4 motors
inputs are from motor positions and x,y,z
"""

#GA use libraries
import numpy as np
import torch
import random
import copy
import matplotlib.pyplot as plt
#libraries for the biped
from adafruit_servokit import ServoKit
from mpu6050 import mpu6050 as MPU
from Bluetin_Echo import Echo
import RPi.GPIO as GPIO
import time
"""
Hardware classes
"""
class servoMotor:
    def __init__(self,servoObj,start,Min,Max):
        self.servo=servoObj
        self.min=Min
        self.max=Max
        self.start=start
    def move(self,angle): #only move within a set space to avoid damage
        current=self.servo.angle
        if current+angle>=self.min and current+angle<=self.max:
            self.servo.angle=current+angle
    def startPos(self):
        self.servo.angle=self.start

class Agent:
    def __init__(self, num_input, num_output):
        self.num_input = num_input 
        self.num_output = num_output
        self.num_genes = num_input * num_output + num_output
        self.weights = None
        self.bias = None

    def set_genes(self, gene):
        weight_idxs = self.num_input * self.num_output
        bias_idxs = self.num_input * self.num_output + self.num_output
        w = gene[0 : weight_idxs].reshape(self.num_output, self.num_input)   
        b = gene[weight_idxs: bias_idxs].reshape(self.num_output,)
        self.weights = torch.from_numpy(w)
        self.bias = torch.from_numpy(b)

    def forward(self, x):
        x = torch.from_numpy(x).unsqueeze(0)
        return torch.mm(x, self.weights.T) + self.bias

    def get_action(self, x):
        items=list(self.forward(x)[0]) #get predictions
        arr=[]
        for i in items:
            if i>0.1: #if over 0.1 its on
                arr.append(1)
            elif i<=-0.1: #if in lower than -0.1 its off
                arr.append(-1)
            else: #if in middle bracket then
                arr.append(0)
        return arr


"""
define needed variables
"""
num_obs = 7 # depends on your environment
num_actions = 4 # depends on your environment
epochs, pop_size, mutation_std = 150, 15, 0.02 #define data for training
#create out agent
agent = Agent(num_obs,  num_actions)

# Create our gene population
gene_pop = []

for i in range(20):
  gene_pop.append(np.random.normal(0, 0.1, (agent.num_genes)))#create


GPIO.setmode(GPIO.BCM)
NumServos=8
GPIO.setwarnings(False)
buzzer=23
#set up sensors
sensor = MPU(0x68)
sonar=Echo(16,12,315) #trigger pin, echo pin, speed


#set up outputs
kit = ServoKit(channels=16)
#lcd = lcddriver.lcd()
GPIO.setup(23,GPIO.OUT) #buzzer on 23

servos=[]
servos.append(servoMotor(kit.servo[0],90,40,130))
servos.append(servoMotor(kit.servo[1],130,20,10))
servos.append(servoMotor(kit.servo[2],100,0,180))
servos.append(servoMotor(kit.servo[3],30,0,80))


"""
define needed functions
"""
def readDist():
    return int(sonar.read("mm",15))
def readAcc():
     while True:
        try:
                d=sensor.get_accel_data() #read the accelerometer
                return float(d["x"]),float(d["y"]),float(d["z"])
        except:
                pass
def withinBoundary(num,value,minus,plus): #whether or not a number is withi a value bounds
    if num>=value-minus and num<=value+plus:
        return True
    return False

def isReady():
    #check accelerometer values are within boundaries
    x,y,z=readAcc()#get gyroscope values
    if withinBoundary(x,-9.5,2,2) and withinBoundary(y,0.2,2,2):
        return True
    return False
def getDiverseScore(positions):
    #given all the positions predicted, reward those with many changes
    counter=0
    for i,pos in enumerate(positions[1:]):
        for j in range(len(positions[0])):
            if positions[i][j]!=pos[j]:
                counter+=1
    return counter
def fitness(startDist,positions):
    #get the fitness of the bots current position
    distance=readDist()
    diversityScore=getDiverseScore(positions) #gather a diversity of movement score
    x,y,z=readAcc()
    #get the distance score
    #get the gyro score
    #combine scores
    penalty=0
    if not withinBoundary(x,-9.5,2,2):
        penalty=abs(max(-9.5,x)-min(-9.5,x))
    if not withinBoundary(y,0.2,2,2):
        penalty=abs(max(0.2,x)-min(0.2,x))
    
    if isReady(): #if in a near position
        if startDist-distance>0 and startDist-distance-penalty>0:
            return (startDist-distance-penalty)+diversityScore
    return 0

def mutation(gene, mean=0, std=0.1):
    gene = gene + np.random.normal(mean, std, size=gene.shape)
    # constraint
    gene[gene > 4] = 4
    gene[gene < -4] = -4
    return gene

def output_step(servos,motorGenes): #move servos by given amounts
    for i,gene in enumerate(motorGenes):
        if gene==1:
            servos[i].move(30) #move by 30 degrees
        elif gene==-1:
            servos[i].move(-30) #move by -30 degrees
        
prev_fitness = [0]
fitnesses=[0]
print("initial reading",readDist())
# Main loop performing Microbal GA
for epoch in range(epochs):
    print("Generation:",epoch)
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    time.sleep(2)
    startDist=readDist() #get sensor reading
    
    n1=random.randint(0,19) #get random gene
    g1=gene_pop[n1]
    g1=mutation(g1,std=0.2)
    positions=agent.set_genes(g1)
    currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
    gathered=[]
    for i in range(20): #20 steps to get it right
        positions=agent.get_action(np.array(currentMotors+list(readAcc()))) #get random gene
        gathered.append(positions.copy())
        currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
        output_step(servos,positions) ######output steps
        time.sleep(0.3)
        if not isReady(): break
    g1_fit = fitness(startDist,gathered)
    
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    time.sleep(2)
    startDist=readDist() #get sensor reading
    
    n2=random.randint(0,19)
    g2=gene_pop[n2]
    g2=mutation(g2,std=0.2)
    positions=agent.set_genes(g2)
    currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
    gathered=[]
    for i in range(20): #20 steps to get it right
        positions=agent.get_action(np.array(currentMotors+list(readAcc()))) #get random gene
        gathered.append(positions.copy())
        currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
        output_step(servos,positions) ######output steps
        time.sleep(0.3)
        if not isReady(): break
    g2_fit = fitness(startDist,gathered)
    
    if g1_fit>g2_fit: gene_pop[n2]=copy.deepcopy(gene_pop[n1]) #copy over
    else: gene_pop[n1]=copy.deepcopy(gene_pop[n2])
    
    fitnesses.append(max([g1_fit,g2_fit])) #save best fitness out of two
    print(fitnesses)

import datetime
file=open("/home/pi/Documents/Walking/dataSheet Microbal"+str(datetime.datetime.now()).replace(":","")+".txt","w")
#file.write(str(gt[t_ind].genotype))
file.write(str(fitnesses))
file.close()
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)

sonar.stop()
