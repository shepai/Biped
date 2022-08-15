"""
Genetic algorithm experiment
By Dexter R C Shepherd,aged 19

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
import copy

GPIO.setmode(GPIO.BCM)
NumServos=4
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
        self.fitnesses=[0 for i in range(size)]
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
                new[i]=copy.deepcopy(seq)
        return new
    def setNew(self,geno,index=-1): #set the new genotype as the parameter
        if index>=0:
            self.genotype[0:index]=copy.deepcopy(geno[0:index])
        else:
            self.genotype=copy.deepcopy(geno)
    def setFitness(self,index,num): #set the fitnesses of each step
        self.fitnesses[index]=num

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
def readDist(): #get the distance of the bot
    dist=sonar.read("mm",5)
    return dist

def MixGenes(genotype1,genotype2):
    #geth both genes and find the half of genotype with the best fitness
    #merge the genotypes and return two new genotypes
    gt1=current1.genotype
    ft1=current1.fitnesses
    gt2=current2.genotype
    ft2=current2.fitnesses
    mid=int((len(ft1))/2)
    a=[]
    b=[]
    if max(ft1[0:mid])>max(ft1[mid-1:-1]): #gather best half
        a=gt1[0:mid]
    else:
        a=gt1[mid-1:-1]
    if max(ft2[0:mid])>max(ft2[mid-1:-1]): #gather best half
        b=gt2[0:mid]
    else:
        b=gt2[mid-1:-1]
    new=a+b
    genotype1.setNew(new) #set the new genotype
    return genotype1,genotype1 #return children
##################
#set up everything else
servos=[]
servos.append(servoMotor(kit.servo[0],90,40,130))
servos.append(servoMotor(kit.servo[1],130,20,10))
servos.append(servoMotor(kit.servo[2],100,0,180))
servos.append(servoMotor(kit.servo[3],30,0,80))

gt=[Genotype(size=30,no_of_inputs=NumServos,options=[0,0,30,-30,0,0,0]) for i in range(30)]


fittnesses=[]
##################
#Begin algorithm

Generations=200
top=0
best=0
t_ind=0
for gen in range(Generations):
    print("Generation",gen+1)
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    startDist=readDist() #get sensor reading
    n1=random.randint(0,29)
    current1=gt[n1].mutate(rate=0.2)
    for j,task in enumerate(current1): #perform genotype encoding
        for i,ang in enumerate(task):
            servos[i].move(ang) #move each servo
        current1.setFitness(j,fitness(startDist)) #set fitness at each step
        time.sleep(0.2)
        if isReady(): #make sure the ot has not fallen over
            topInd=j
            bestInBatch=fitness(startDist)
        else:
            break
    fit1=fitness(startDist) #gather end fitness
    
    for i in servos:
        i.startPos()
    while isReady()==False: GPIO.output(buzzer,GPIO.HIGH) #wait for ready
    GPIO.output(buzzer,GPIO.LOW)
    startDist=readDist() #get sensor reading
    n2=random.randint(0,29)
    current2=gt[n2].mutate(rate=0.2)
    for j,task in enumerate(current2): #perform genotype encoding
        for i,ang in enumerate(task):
            servos[i].move(ang) #move each servo
        time.sleep(0.2)
        current1.setFitness(j,fitness(startDist)) #set fitness at each step
        if isReady(): #make sure the bot has not fallen over
            topInd=j
            bestInBatch=fitness(startDist)
        else:
            break
    fit2=fitness(startDist) #gather end fitness
    t=max(fit1,fit2)
    fittnesses.append(t)
    #merge genotypes to create new offspring
    current1,current2=MixGenes(current1,current2)
    gt[n2].setNew(current2)
    gt[n1].setNew(current1)
    if t>=top: t_ind=n2
#################
#Save information
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
GPIO.output(buzzer,GPIO.HIGH) 
time.sleep(2)
GPIO.output(buzzer,GPIO.LOW)
time.sleep(1)
file=open("/home/pi/Documents/Walking/dataSheet Microbal.txt","w")
file.write(str(gt[t_ind].genotype))
file.write(str(fittnesses))
file.close()
sonar.stop()

while True:
    time.sleep(1)
    for i in servos:
        i.startPos()
    for j,task in enumerate(gt[t_ind].genotype):
        for i,ang in enumerate(task):
            servos[i].move(ang)
        time.sleep(0.2)


