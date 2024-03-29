"""
Genetic algorithm experiment
By Dexter R Shepherd,aged 19

This uses a genetic algorithm to move a biped chassis of selected size. It should optimize its walk by using
an mpu6050 sensor and ultrasonic range finder.

PINOUT:
*trigger pin 16
*echo pin 12
*buzzer 23

"""
#https://learn.adafruit.com/ultrasonic-sonar-distance-sensors/python-circuitpython
#import lcddriver
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
    def __init__(self,size=20,no_of_inputs=1,options=[]):
        self.size=size
        self.no_of_inputs=no_of_inputs
        self.options=options
        self.genotype=[[random.choice(options) for i in range(no_of_inputs)] for j in range(size)]
    def mutate(self,rate=0.2): #get a mutated genotype
        new=self.genotype.copy()
        for i in range(self.size): #loop through mutation rate times
            #get random positions
            if random.random()<=rate: #give mutation rate chance
                seq=new[i]
                n=random.randint(0,len(seq)-1)
                choice=random.choice(self.options)
                while choice==seq[n]:  choice=random.choice(self.options) #get unique
                #reform in order
                seq[n]=choice
                new[i]=seq.copy()
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


gt=Genotype(size=30,no_of_inputs=NumServos,options=[0,0,20,-20,0,0,0,0])

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
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    startDist=readDist() #get sensor reading
    #lcd.lcd_display_string("Generation "+str(gen+1), 3)
    current=gt.mutate(rate=0.2)
    #perform genotype
    for task in current:
        for i,ang in enumerate(task):
            servos[i].move(ang)
        time.sleep(0.5)
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
