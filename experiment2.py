"""
Genetic algorithm experiment
By Dexter R C Shepherd,aged 19
"""
#https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/python-circuitpython
#import lcddriver
import random
from time import *
from adafruit_servokit import ServoKit
from mpu6050 import mpu6050 as MPU
from Blueton_Echo import Echo

#set up outputs
kit = ServoKit(channels=16)
#lcd = lcddriver.lcd()

#set up sensors
mpu = MPU(0x68)
sonar=Echo(16,12,315) #trigger pin, echo pin, speed

"""
lcd.lcd_display_string("GA experiment 2", 1)
lcd.lcd_display_string("By Dexter Shepherd", 2)
lcd.lcd_display_string("", 3)
lcd.lcd_display_string("shepai.github.io", 4)
sleep(2)
"""
##################
#Define relevant classes and functions

class servoMotor:
    def __init__(self,servoObj,Min,Max):
        self.servo=servoObj
        self.min=Min
        self.max=Max
    def move(angle): #only move within a set space to avoid damage
        current=self.servo.angle
        if current+angle>=self.min and current+angle<=self.max:
            self.servo.angle=current+angle
class Genotype:
    def __init__(self,size=20,mutations=1,no_of_inputs=1,options=[]):
        self.size=size
        self.mutationRate=mutations
        self.no_of_inputs=no_of_inputs
        self.options=options
        self.genotype=[[random.choice(options) for i in range(no_of_inputs)] for j in range(size)]
    def mutate(self): #get a mutated genotype
        new=self.genotype.copy()
        for i in range(self.mutationRate): #loop through mutation rate times
            #get random positions
            n=random.randint(0,self.size)
            seq=self.genotype[n]
            n1=random.randint(0,len(seq))
            choice=random.choice(self.options)
            while choice==seq[n1]:  choice=random.choice(self.options) #get unique
            #reform in order
            seq[n1]=choice
            new[n]=seq.copy()
        return new
    def setNew(self,geno): #set the new genotype as the parameter
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
def Set(positions):
    for i,ang in enumerate(positions): #loop through the positions and find
        servos[i].servo.angle=ang
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
startPositions=[]
for i in range(8): #create all servos and their parameters
    servos.append(servoMotor(kit.servo[i],0,180))

gt=Genotype(size=30,mutations=3,no_of_inputs=8,options=[0,0,20,-20,0,0,0,0])

fittnesses=[]
##################
#Begin algorithm
"""
lcd.lcd_display_string("[][][][]    [][][][]", 1)
lcd.lcd_display_string("[][][][]    [][][][]", 2)
lcd.lcd_display_string("", 3)
lcd.lcd_display_string("shepai.github.io", 4)
"""
Generations=50
best=0
for gen in range(Generations):
    #lcd.lcd_display_string("", 3)
    Set(startPositions)
    startDist=readDist() #get sensor reading
    while isReady()==False: pass #wait for ready 
    #lcd.lcd_display_string("Generation "+str(gen+1), 3)
    current=gt.mutate()
    fit=fitness(startDist)
    fittnesses.append(fit)
    if fit>best:
        gt.setNew(current)
        best=fit
#################
#Save information

file=open("dataSheet ","w")
file.write(str(gt.genotype))
file.write(str(fittnesses))
file.close()
sonar.stop()
