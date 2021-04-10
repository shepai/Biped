

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
        #print(self.forward(x))
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
num_obs = 5 # depends on your environment
num_actions = 4 # depends on your environment
epochs, pop_size, mutation_std = 1000, 15, 0.01 #define data for training
#create out agent
agent = Agent(num_obs, 5,  num_actions)

# Create our gene population
gene_pop = []
for i in range(20):
  gene_pop.append(np.random.normal(0, 0.1, (pop_size, agent.num_genes)))#create


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
"""
define needed functions
"""
def readAcc():
    d=sensor.get_accel_data() #read the accelerometer
    return float(d["x"]),float(d["y"]),float(d["z"])
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
def fitness(startDist):
    #get the fitness of the bots current position
    distance=readDist()
    x,y,z=readAcc()
    #get the distance score
    #get the gyro score
    #combine scores
    penalty=0
    if not withinBoundary(x,-9.5,0.5,0.5):
        penalty=abs(max(-9.5,x)-min(-9.5,x))
    if not withinBoundary(y,0.2,0.8,1):
        penalty=abs(max(0.2,x)-min(0.2,x))
    if withinBoundary(x,-9.5,2,2) and withinBoundary(y,0.2,2,2): #if in a near position
        if startDist-distance>0 and startDist-distance+penalty>0:
            return startDist-distance+penalty
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
fitnesses=[]

# Main loop performing Microbal GA
for epoch in range(epochs):
    print("Generation:",epoch)
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    startDist=readDist() #get sensor reading
    
    n1=random.randint(0,19) #get random gene
    g1=gene_pop[n1]
    positions=agent.set_genes(g1)
    currentMotors=[servo[i].servo.angle for i in range(len(servos))] #set up current angles
    for i in range(20): #20 steps to get it right
        positions=agent.get_action(currentMotors+list(readAcc())) #get random gene
        currentMotors=[]
        output_step(servos,positions) ######output steps
        g1_fit = fitness(startDist)
    
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    
    startDist=readDist() #get sensor reading
    
    n2=random.randint(0,19)
    g2=gene_pop[n2]
    positions=agent.set_genes(g2)
    currentMotors=[servo[i].servo.angle for i in range(len(servos))] #set up current angles

    for i in range(20): #20 steps to get it right
        positions=agent.get_action(currentMotors+list(readAcc())) #get random gene
        currentMotors=[]
        output_step(servos,positions) ######output steps
        g2_fit = fitness(startDist)
    
    if g1_fit>g2_fit: gt[n2]=gt[n1].copy() #copy over
    else: gt[n1]=gt[n2].copy()
    
    fitnesses.append(max([g1_fit,g2_fit,max(fitnesses)])) #save best fitness out of two

GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
import datetime
file=open("/home/pi/Documents/Walking/dataSheet Microbal"+str(datetime.datetime.now())+".txt","w")
file.write(str(gt[t_ind].genotype))
file.write(str(fittnesses))
file.close()
sonar.stop()


        
