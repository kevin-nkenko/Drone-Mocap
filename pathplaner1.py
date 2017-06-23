#-*- coding: utf-8 -*-


from visual import *
from droneModel import *
import random


X1=[100,200,250]
Y1=[100,200,300]
Z1=[350,400,280]


for i in range(15):
    X1 += [X1[-1] + int(random.uniform(0,200))]
    Y1 += [Y1[-1] + int(random.uniform(0,200))]
    Z1 += [Z1[-1] + int(random.uniform(0,200))]
for i in range(10):
    X1 += [X1[-1] + int(random.uniform(0,200))]
    Y1 += [Y1[-1] + int(random.uniform(0,200))]
    Z1 += [Z1[-1] - int(random.uniform(0,200))]




monDrone = Drone()
monDrone.takeoff()
pas=50

for i in range(len(X1)):
    monDrone.goToPoint1([X1[i], Y1[i], Z1[i]], pas)

X = monDrone.goneToX
Y = monDrone.goneToY
Z = monDrone.goneToZ

X2 = [ monDrone.positionX[i] for i in range (1, len(monDrone.positionX), 1000)]
Y2 = [ monDrone.positionY[i] for i in range (1, len(monDrone.positionY), 1000)]
Z2 = [ monDrone.positionZ[i] for i in range (1, len(monDrone.positionZ), 1000)]
del monDrone

print(X1)
print(Y1)
print(Z1)
print(X)
print(Y)
print(Z)
print(Y2)


tracerTrajectoire(X,Y, Z, X1, Y1, Z1)
animerPoints(X2,Y2,Z2)

