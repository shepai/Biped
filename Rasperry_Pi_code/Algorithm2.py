"""
GA pre-programmed sequence

By Dexter R Shepherd, aged 19

This uses a genetic algorithm to move a biped chassis of selected size. It should optimize its walk by using
an mpu6050 sensor and ultrasonic range finder.

PINOUT:
*trigger pin 16
*echo pin 12
*buzzer 23


"""

import random
import time
from adafruit_servokit import ServoKit
from mpu6050 import mpu6050 as MPU
from Bluetin_Echo import Echo
import RPi.GPIO as GPIO

import random
import time
from adafruit_servokit import ServoKit
from mpu6050 import mpu6050 as MPU
from Bluetin_Echo import Echo
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
NumServos=8
GPIO.setwarnings(False)
buzzer=23

#set up outputs
kit = ServoKit(channels=16)
#lcd = lcddriver.lcd()
GPIO.setup(23,GPIO.OUT) #buzzer on 23

#set up sensors
sensor = MPU(0x68)
sonar=Echo(16,12,315) #trigger pin, echo pin, speed

##################
#Define relevant classes and functions

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
class Genotype:
    def __init__(self,size=20,no_of_inputs=1,sequence=[]):
        self.size=size
        self.no_of_inputs=no_of_inputs
        self.genotype=sequence
        try:
            if type(self.genotype[0][0])!=type(0):
                raise ValueError("Invalid values within sequence")
            if len(self.genotype)!=size:
                raise EnvironmentError("Sequence should be same size as size")
        except IndexError:
            raise TypeError("Sequence should be of array of array of integers")
        #set up genotype
    def mutate(self,rate=0.2): #get a mutated genotype
        new=self.genotype.copy()
        for i in range(self.size): #loop through mutation rate times
            #get random positions
            if random.random()<=rate: #give mutation rate chance
                seq=new[i]
                n=random.randint(0,len(seq)-1)
                while seq[n]==0: n=random.randint(0,len(seq)-1)
                choice=randint(-30,30)
                #reform in order
                seq[n]=choice
                new[i]=seq.copy()
        return new
    def setNew(self,geno,index=-1): #set the new genotype as the parameter
        if index>=0:
            self.genotype[0:index]=geno[0:index]
        else:
            self.genotype=geno.copy()
        
def readGyro():
    return sensor.get_accel_data() #read the gyroscope
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
    if withinBoundary(x,-9.5,0.5,0.5) and withinBoundary(y,0.2,0.8,1):
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
def readDist(): #get the distance of the bot
    dist=sonar.read("mm",5)
    return dist
##################
#set up everything else
servos=[]
servos.append(servoMotor(kit.servo[0],90,60,180))
servos.append(servoMotor(kit.servo[1],0,0,180))
servos.append(servoMotor(kit.servo[2],100,0,180))
servos.append(servoMotor(kit.servo[3],130,0,180))
servos.append(servoMotor(kit.servo[4],90,0,130))
servos.append(servoMotor(kit.servo[5],100,0,180))
servos.append(servoMotor(kit.servo[6],180,0,180))
servos.append(servoMotor(kit.servo[7],90,0,180))

gt=[]
gt.append([0,0,0,0,0,0,0,30])
gt.append([0,10,0,0,0,20,0,0])
gt.append([0,0,0,0,0,10,0,0])
gt.append([0,10,0,-10,0,0,0,0])
gt.append([0,0,0,0,0,-10,0,0])
gt.append([0,-20,0,0,0,0,0,-30])


gt=Genotype(size=6,no_of_inputs=NumServos,sequence=gt)

fittnesses=[]
##################
#Begin algorithm

Generations=50
best=0
for gen in range(Generations):
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    time.sleep(2)
    startDist=readDist() #get sensor reading
    current=gt.mutate(rate=0.2)
    #perform genotype
    topInd=-1
    bestInBatch=0
    for j,task in enumerate(current):
        for i,ang in enumerate(task):
            servos[i].move(ang)
        time.sleep(0.5)
        if isReady():
            topInd=j
            bestInBatch=fitness(startDist)
        else:
            break
    fit=fitness(startDist)
    fittnesses.append(fit)
    if fit>best:
        gt.setNew(current)
        best=fit
    elif bestInBatch>best: #store where it was good up to the point
        gt.setNew(current,index=topInd)
        
#################
#Save information

file=open("dataSheet ","w")
file.write(str(gt.genotype))
file.write(str(fittnesses))
file.close()
sonar.stop()
