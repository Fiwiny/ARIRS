"""XXrobot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from shutil import move
from controller import Robot
from controller import Motor
from controller import GPS
from controller import Motion
from controller import InertialUnit

import time
import _thread
import math
import numpy as np
# create the Robot instance.
robot = Robot()
def vec(me, you):
    vecget = np.array(you)-np.array(me)
    return vecget.tolist()


def costheta(me, forward, aim):
    vecm2f = np.array(vec(me, forward))
    vecm2a = np.array(vec(me, aim))
    # print("vecm2f",vecm2f)
    # print("vecm2a",vecm2f)
    costhe = np.multiply(vecm2f, vecm2a).sum(
    )/(np.sqrt((vecm2f**2).sum())*np.sqrt((vecm2a**2).sum()))
    # print("costhe",costhe)
    return costhe
    
    
def turnlef(vecm2f, vecm2a):
    lab = vecm2f[0]*vecm2a[2]-vecm2f[2]*vecm2a[0]
    # print("lab",lab)
    if lab < 0:
        return True
    else:
        return False
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)
gps1 = robot.getDevice("gps(1)")
gps1.enable(timestep)

wheel1=robot.getDevice('wheel1')
wheel2=robot.getDevice('wheel2')
wheel3=robot.getDevice('wheel3')
wheel4=robot.getDevice('wheel4')
wheel5=robot.getDevice('wheel5')
wheel6=robot.getDevice('wheel6')
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))
wheel3.setPosition(float('inf'))
wheel4.setPosition(float('inf'))
wheel5.setPosition(float('inf'))
wheel6.setPosition(float('inf'))

leftvel=1
rightvel=1
cleaner=0

wheel1.setVelocity(leftvel)
wheel2.setVelocity(leftvel)
wheel3.setVelocity(rightvel)
wheel4.setVelocity(rightvel)
wheel5.setVelocity(cleaner)
wheel6.setVelocity(0)


dsback=robot.getDevice('DS_back')
dsback.enable(timestep)
ds=robot.getDevice('DS')
ds.enable(timestep)
camcle=robot.getDevice('cam1')
camcle.enable(timestep)
camcle.recognitionEnable(10)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller


aimlist = [[-2,0,0.0497],[1,1.2,0.0497]]

aimcounter = 0

forpos = 0
countstep = 0
turn180 = 0 # 防止180度角的震荡
turnon180 = 0

Counter = 0
then = 0
time=2000
repairing = 0

while robot.step(timestep) != -1:
    countstep += 1
    aim=aimlist[aimcounter]
    pos = gps.getValues() # 车头的gps
    pos1 = gps1.getValues() # 车尾的gps
    cos = costheta(pos1, pos, aim) # 夹角余弦值
    vecm2f = np.array(vec(pos1, pos)) # 车子向量
    vecm2a = np.array(vec(pos1, aim)) # 车尾到目标向量
    tlf = turnlef(vecm2f,vecm2a) # 判断是否左转
    #初始化速度
    leftvel = 1.0
    rightvel = 1.0
    numObjects = camcle.getRecognitionNumberOfObjects()
    aimdis = math.sqrt(sum([(aim - pos)**2 for (aim,pos) in zip(aim,pos)]))
     #print("forpos",forpos)
     #print("aimdis",aimdis)
    
    if aimdis < 0.3:
        leftvel = 0
        rightvel = 0
        wheel6.setVelocity(1)
        if aimcounter >= (len(aimlist)-1):
            print("end moving")
            wheel5.setVelocity(0)           
            wheel6.setVelocity(0)
            
        else:
            print("continue moving")
            aimcounter = aimcounter+1
            aim = aimlist[aimcounter]
            
    elif Counter > 0:
        if turnon180 > 0:
            leftvel = 0.5*turn180
            rightvel = -0.5*turn180
            turnon180 -= 1
        #
        if Counter >1:            
            if tlf:
                leftvel = 0.5
                rightvel = -0.5
            else:
                leftvel = -0.5
                rightvel = 0.5
        else:
            then = 33
            Counter -= 1

    elif (1 - cos)>(0.085/aimdis):
        if then>0:
            then -= 1
            leftvel = 0.5
            rightvel = 0.5
        else:
            # print("tlf",tlf)
            if tlf:
                leftvel = -0.5
                rightvel = 0.5
            else:
                leftvel = 0.5
                rightvel = -0.5
        
    # else:  # read sensors
    if turnon180 <= 0:
        if (1 - cos)>1.9:                
            if tlf:
                turn180 = 1
            else:
                turn180 = -1
            turnon180 = 150
            Counter = 150

    ds1=ds.getValue()
    ds2=dsback.getValue()
    
    if ds1>300 or ds2>510:
         print('front edge detected')
         print('stop')
         wheel1.setVelocity(0)
         wheel2.setVelocity(0)
         wheel3.setVelocity(0)
         wheel4.setVelocity(0)
         wheel5.setVelocity(0)
         wheel6.setVelocity(0)
         break




    # dsright_val=dsright.getValue()
    # dsleft_val=dsleft.getValue()

    # image = camcle.getImage()
    image = camcle.getImageArray()
    red=0
    green=0
    blue=0
    # time = robot.getTime()
    ts=robot.getTime()
    for x in range(0,camcle.getWidth()):
       for y in range(0,camcle.getHeight()):
           red   += image[x][y][0]
           green += image[x][y][1]
           blue  += image[x][y][2]
    gray  = (red + green + blue) / 3
    
    # if str(time).isnumeric():
    if gray<850000:
               time = robot.getTime()

               wheel5.setVelocity(10) 
    if gray>850000:
         if time+4<ts:

               wheel5.setVelocity(0)
    # else:wheel5.setVelocity(0)         
    # elif gray >300000:
          # wheel5.setVelocity(0) 

    # elif pos==[0.5,0.5,0.0497]:
         # wheel6.setVelocity(1)
    #print(ds.getValue(),gray,numObjects,time,ts,green,aimcounter)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
                       
    #设置电机速度
    forpos = aimdis
    wheel1.setVelocity(leftvel)
    wheel2.setVelocity(leftvel)
    wheel3.setVelocity(rightvel)
    wheel4.setVelocity(rightvel)
    pass

# Enter here exit cleanup code.
