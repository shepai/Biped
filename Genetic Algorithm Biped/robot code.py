
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
    def __init__(self,servoObj,start,Min,Max): #initialize with all values and constraints
        self.servo=servoObj
        self.min=Min
        self.max=Max
        self.start=start
    def move(self,angle): #only move within a set space to avoid damage
        current=self.servo.angle
        if current+angle>=self.min and current+angle<=self.max:
            self.servo.angle=current+angle #increase by only if within constraint
    def startPos(self): #set servo to the start position (which will be standing)
        self.servo.angle=self.start

class Agent:
    def __init__(self, num_input, num_hiddenLayer, num_output):
        self.num_input = num_input  #set input number
        self.num_output = num_output #set ooutput number
        self.num_genes = (num_input * num_hiddenLayer) + (num_output) + (num_hiddenLayer*num_output)
        self.num_hidden=num_hiddenLayer
        self.weights = None
        self.weights2=None
        self.bias = None

    def set_genes(self, gene):
        weight_idxs = self.num_input * self.num_hidden #size of weights to hidden
        weights2_idxs = self.num_hidden * self.num_output + weight_idxs #size and position
        bias_idxs = weight_idxs + weights2_idxs + self.num_output #sizes of biases
        w = gene[0 : weight_idxs].reshape(self.num_hidden, self.num_input)   #merge genes
        w2 = gene[weight_idxs : weights2_idxs].reshape(self.num_output, self.num_hidden)   #merge genes
        b = gene[weights2_idxs: bias_idxs].reshape(self.num_output,) #merge genes
        self.weights = torch.from_numpy(w) #assign weights
        self.weights2 = torch.from_numpy(w2) #assign weights
        self.bias = torch.from_numpy(b) #assign biases

    def forward(self, x):
        x = torch.from_numpy(x).unsqueeze(0)
        x=torch.mm(x, self.weights.T) #first layer
        return torch.mm(x,self.weights2.T) + self.bias #secon layer
        

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
agent = Agent(num_obs, 5,  num_actions)

# Create our gene population
gene_pop = []

for i in range(10): #vary from 10 to 20 depending on purpose of robot
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

servos=[] #attach all the servo motor objects
servos.append(servoMotor(kit.servo[0],90,40,130))
servos.append(servoMotor(kit.servo[1],130,20,10))
servos.append(servoMotor(kit.servo[2],100,0,180))
servos.append(servoMotor(kit.servo[3],30,0,80))


"""
define needed functions
"""
def readDist():
    return int(sonar.read("mm",15)) #read the distance from ultrasound range finder
def readAcc():
     while True:
        try:
                d=sensor.get_accel_data() #read the accelerometer
                return float(d["x"]),float(d["y"]),float(d["z"]) #return values from gryoscopic data
        except:
                pass #return nothing if not plugged in
def withinBoundary(num,value,minus,plus): #whether or not a number is withi a value bounds
    if num>=value-minus and num<=value+plus: #get boundaries
        return True
    return False

def isReady():
    #check accelerometer values are within boundaries
    x,y,z=readAcc()#get gyroscope values
    if withinBoundary(x,-9.5,2,2) and withinBoundary(y,0.2,2,2): #get within bounds of standing
        return True
    return False
def getDiverseScore(positions):
    #given all the positions predicted, reward those with many changes
    counter=0
    for i,pos in enumerate(positions[1:]): #loop through positions
        for j in range(len(positions[0])): #loop through each
            if positions[i][j]!=pos[j]:
                counter+=1 #increase if different to last
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
    if not withinBoundary(x,-9.5,2,2): #assign penalties for data not within boundary
        penalty=abs(max(-9.5,x)-min(-9.5,x))
    if not withinBoundary(y,0.2,2,2):
        penalty=abs(max(0.2,x)-min(0.2,x))
    
    if isReady(): #if in a near position
        if startDist-distance>0 and startDist-distance-penalty>0:
            return (startDist-distance-penalty)+diversityScore
    return 0

def mutation(gene, mean=0, std=0.1):
    gene = gene + np.random.normal(mean, std, size=gene.shape) #mutate the gene via normal 
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
fittest=[0,0]
storedFav=-1
print("initial reading",readDist())
# Main loop performing Microbal GA
behaviour=[[0],[0],[0],[0]]
for epoch in range(epochs):
    print("Generation:",epoch)
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    time.sleep(2)
    startDist=readDist() #get sensor reading
    
    n1=random.randint(0,19) #get random gene
    if epoch==149:
        n1=storedFav
    g1=copy.deepcopy(gene_pop[n1])
    g1=mutation(g1,std=0.2) #mutate this random gene
    positions=agent.set_genes(g1) #set the genes
    currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
    gathered=[]
    for i in range(20): #20 steps to get it right
        positions=agent.get_action(np.array(currentMotors+list(readAcc()))) #get random gene
        gathered.append(positions.copy())
        currentMotors=[servos[i].servo.angle for i in range(len(servos))] #set up current angles
        if epoch==75 or epoch==149: #only save for two
            storedFav=fittest[1]
            [behaviour[i].append(servos[i].servo.angle) for i in range(len(servos))]
        output_step(servos,positions) ######output steps
        time.sleep(0.3)
        if not isReady(): break #break if not standing up
    g1_fit = fitness(startDist,gathered) #gather the fitness
    
    for i in servos: #reset to start positions
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    time.sleep(2)
    startDist=readDist() #get sensor reading
    
    n2=random.randint(0,19) #select a random gene
    g2=copy.deepcopy(gene_pop[n2])
    g2=mutation(g2,std=0.2) #mutate this random gene
    positions=agent.set_genes(g2) #set the genes
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
    
    if g1_fit>g2_fit: gene_pop[n2]=copy.deepcopy(gene_pop[n1]) #copy over if fitter
    else: gene_pop[n1]=copy.deepcopy(gene_pop[n2]) #copy over if not fitter
    
    fitnesses.append(max([g1_fit,g2_fit])) #save best fitness out of two
    if max(g1_fit,g2,fit)>fittest[0]: #store the best fitness
        fittest[0]=max(g1_fit,g2,fit)
        if fittest[0]==g1_fit: fittest[1]=n1
        else: fittest[1]=n2
    print(fitnesses)

import datetime
file=open("/home/pi/Documents/Walking/dataSheet Microbal"+str(datetime.datetime.now()).replace(":","")+".txt","w") #save file
#file.write(str(gt[t_ind].genotype))
file.write(str(fitnesses))
file.close()

file=open("/home/pi/Documents/Walking/dataSheet Behaviour"+str(datetime.datetime.now()).replace(":","")+".txt","w") #save file
#file.write(str(gt[t_ind].genotype))
file.write(str(behaviour))
file.close()

GPIO.output(buzzer,GPIO.HIGH)  #alert user that the program has finished with beeps
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)

sonar.stop() #stop sonar
