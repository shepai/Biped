"""

Servo hexapod library

"""
import copy
import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def get_distance(p1,p2): #measure distance between points
    y1,y2=p1[1],p2[1]
    x1,x2=p1[0],p2[0]
    dist = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    return dist
        
class point:
    def __init__(self,x,y):
        """
        @param x coord
        @param y coord
        """
        self.x=x
        self.y=y
        self.outer=[(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        self.colours=["r","r","g","g"]
        self.connected=None
        self.show()
    def rotate(self,angle): #rotate around angle
        a=[]
        for point in self.outer:
            a.append(rotate((self.x,self.y),point,angle))
        self.outer=copy.deepcopy(a)
        conn=self.connected
        while conn!=None:
            x=conn.x
            y=conn.y
            conn.x,conn.y=rotate((self.x,self.y),(x,y),angle)
            plt.plot([self.x,x], [self.y,y],c="b")
            po=conn.outer
            a=[]
            for point in po:
                a.append(rotate((self.x,self.y),point,angle))
            conn.outer=copy.deepcopy(a)
            conn.show()
            conn=conn.connected #next movement
        self.show()
    def connect(self,point):
        plt.plot([self.x,point.x],[self.y,point.y],c="b")
        self.connected=point
    def show(self):
        plt.scatter(self.x,self.y)
        c=0
        for x,y in self.outer:
            plt.plot([self.x,x], [self.y,y],c=self.colours[c])
            plt.scatter(x,y,c=self.colours[c])
            c+=1
        if self.connected!=None:
            plt.plot([self.x,self.connected.x], [self.y,self.connected.y],c="b")
class servo:
    def __init__(self,point,board,ID,Min=0,Max=180,ang=90):
        """
        @param point holding the point information
        @param board connected to the adafruit servo board
        @param ID of the index of th servo
        @param Min for the minimum value the leg can go to
        @param Max for the maximum value the leg can go to
        @param ang for the start angle
        """
        self.id=ID
        self.point=point
        self.board=board
        self.angle=ang
        self.min=Min
        self.max=Max
        self.start=ang
    def calibrate(self,default=0):
        """
        @param default to give the position of a servo when in the centre
        """
        #check if servo datafile exists
        #create one if not
        #set angle to standing
        pass
    def move(self,angle):
        print("Move:",angle)
        if angle>=self.min and angle<=self.max: #check in bounds
            self.point.rotate(math.radians(self.angle-angle))
            self.angle=angle
        else:
            if angle>self.angle:
                print(self.angle,self.max-self.angle)
                self.point.rotate(math.radians(self.max-self.angle))
                self.angle=self.max
            else:
                print(self.angle,self.min-self.angle)
                self.point.rotate(math.radians(self.min*self.angle))
                self.angle=self.min
    def show(self):
        self.point.show()
    def reset(self):
        self.angle=self.start
        self.point.rotate(math.radians(self.angle-self.start))
        
class leg:
    def __init__(self,joints=3,x_pos=10,start_y=2,distance=[10,10]): #create structure
        """
        @param joints for number of
        @param x_pos for starting point
        """
        self.joints=[]
        self.servos=[]
        p=point(x_pos,start_y)
        start_y+=distance[0]
        self.joints.append(p)
        for i in range(1,joints):
            p=point(x_pos,start_y)
            p.connect(self.joints[i-1])
            self.servos.append(servo(self.joints[i-1],None,i))
            self.joints.append(p)
            start_y+=distance[i-1]
        self.servos.append(servo(self.joints[joints-1],None,joints-1))
    def update(self,epoch=0): #update the display
        plt.cla()
        plt.ylim((-50,50))
        plt.xlim((-50,50))
        plt.title(epoch)
        for i in range(len(self.servos)):
            self.servos[i].show() #show each
    def MoveTo(self,x,y): #generate new angles based on coordinates
        tipPoint=[x,y]
         #get lines and coords
        s0x=self.servos[0].point.x #Point two
        s0y=self.servos[0].point.y
        s1x=self.servos[1].point.x #Point two
        s1y=self.servos[1].point.y
        s2x=self.servos[2].point.x #Point one
        s2y=self.servos[2].point.y
        if s2x==tipPoint[0]: tipPoint=[s2x+0.1,tipPoint[1]]
        d3=get_distance([s2x,s2y],tipPoint) #distance from p1 to p3
        d1=10#get_distance([s2x,s2y],[s1x,s1y]) #distance from p1 to p3
        d2=10#get_distance([s2x,s2y],[s1x,s1y]) #distance from p1 to p3
        b1=get_distance([s2x,s2y],[s0x,s0y]) #distance from p1 to p3

        #current triangle
        midpoint1=((s2x+s0x)/2,(s2y+s0y)/2)
        h=get_distance([s2x,s2y],midpoint1)
        area=(b1*h)/2

        #gather new h
        h=math.sqrt(abs(d1**2 - (d3/2)**2))
        #h=(d1*h)/d3
        #gather line equation
        m=(s2y-tipPoint[1])/(s2x-tipPoint[0])
        nm=-1/m
        midpoint=((s2x+tipPoint[0])/2,(s2y+tipPoint[1])/2)
        b=midpoint[1]-(nm*midpoint[0])
        n=5
        if n==midpoint[0]: n=4 #prevent error from same point
        y=(nm*n)+b #calculate new pair of coords
        
        #vector (5,y) and midpoint
        v=(n-midpoint[0],y-midpoint[1]) #get vector
        ux=v[0]/(math.sqrt(v[0]**2 + v[1]**2)) #xvec
        uy=v[1]/(math.sqrt(v[0]**2 + v[1]**2)) #yvec
        p2x=midpoint[0]-(h*ux) #x point
        p2y=midpoint[1]-(h*uy) #y point

        #cosine rotate joint
        d4=get_distance([s1x,s1y],[p2x,p2y]) #distance from p1 to p3
        Theta=math.acos((d4**2 - (d1**2)  - (d2**2))/(-2*d2*d1))
        if p2x+p2y>s1x+s1y: #move to correct side
            #print("back")
            self.servos[2].move(self.servos[2].angle-math.degrees(Theta))
        else:
            #print("forward")
            self.servos[2].move(self.servos[2].angle+math.degrees(Theta))
        l.update(epoch=i)
        #cosine rotate knee
        s0x=self.servos[0].point.x #Point two
        s0y=self.servos[0].point.y
        s1x=self.servos[1].point.x #Point two
        s1y=self.servos[1].point.y
        d5=get_distance([s0x,s0y],randPoint) #distance from p1 to p3
        d6=get_distance([s1x,s1y],randPoint) #distance from p1 to p3
        Theta=math.acos((d5**2 - (d2**2) - (d6**2))/(-2*d6*d2))
        
        if s0x>randPoint[0]: #move to correct side
            self.servos[1].move(self.servos[1].angle+(math.degrees(Theta)))
        else:
            self.servos[1].move(self.servos[1].angle-(math.degrees(Theta)))
        self.update()
        plt.plot([s2x,p2x],[s2y,p2y])
        plt.plot([randPoint[0],p2x],[randPoint[1],p2y])
        plt.scatter(randPoint[0],randPoint[1],c="y",s=20)
        plt.scatter(p2x,p2y,c="b",s=20)

        plt.pause(0.05)
    def reset(self):
        for i in self.servos:
            i.reset()
            

l=leg()
plt.ylim((-50,50))
plt.xlim((-50,50))
l.update()

for i in range(800): #perform
    randPoint=(random.randint(0,22),random.randint(2,8))
    plt.scatter(randPoint[0],randPoint[1],c="y",s=20)
    plt.pause(0.05)
    l.update()
    l.MoveTo(randPoint[0],randPoint[1])
   
    time.sleep(1)
    
    l.reset()
plt.show()
