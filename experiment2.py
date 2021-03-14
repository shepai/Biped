"""
Genetic algorithm experiment
By Dexter R C Shepherd,aged 19
"""
#https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/python-circuitpython
import lcddriver
import random
from time import *
import board
import busio
import adafruit_mpu6050
from adafruit_servokit import ServoKit
import adafruit_hcsr04


#set up outputs
kit = ServoKit(channels=16)
lcd = lcddriver.lcd()

#set up sensors
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)
sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)

lcd.lcd_display_string("GA experiment 2", 1)
lcd.lcd_display_string("By Dexter Shepherd", 2)
lcd.lcd_display_string("", 3)
lcd.lcd_display_string("shepai.github.io", 4)
sleep(2)

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
            new[n]=seq[n1].copy()
        return new
    def setNew(self,geno): #set the new genotype as the parameter
        self.genotype=geno.copy()
        
def readGyro():
    pass #read the gyroscope
def readAcc():
    pass #read the accelerometer
def withinBoundary(num,value,minus,plus): #whether or not a number is withi a value bounds
    if num>=value-minus and num<=value+plus:
        return True
    return False
def Set(positions):
    for i,ang in enumerate(positions): #loop through the positions and find
        servos[i].servo.angle=ang
def isReady():
    #check accelerometer values are within boundaries
    ax=readGyro()#get gyroscope values
    if withinBoundary(ax[0],10,5,5):
        return True
    return False
def fitness():
    #get the fitness of the bots current position
    distance=readDist()
    ax=readGyro()
    #get the distance score
    #get the gyro score
    #get the accelerometer score
    #combine scores
    if isReady()==False:
        return 0
    return 1
def readDist(): #get the distance of the bot
    dist=0
    while True:
        try:
            dist=sonar.distance
            break
        except RuntimeError:
            pass
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
lcd.lcd_display_string("[][][][]    [][][][]", 1)
lcd.lcd_display_string("[][][][]    [][][][]", 2)
lcd.lcd_display_string("", 3)
lcd.lcd_display_string("shepai.github.io", 4)

Generations=50
best=0
for gen in range(Generations):
    lcd.lcd_display_string("", 3)
    Set(startPositions)
    while isReady()==False: pass #wait for ready 
    lcd.lcd_display_string("Generation "+str(gen+1), 3)
    current=gt.mutate()
    fit=fitness()
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
