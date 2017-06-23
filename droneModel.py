#-*- coding: utf-8 -*-


import time
import math
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib import animation


def distanceXY(point1, point2):
    """
    Distance des projections dans le plan XOY
    """
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def distance(point1, point2):
    """
    Distance dans l'espace
    """
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 +  (point1[2]-point2[2])**2)

def denivellation(point1, point2):
    """
    Différence de hauteur, c'est à dire distance des projections sur l'axe OZ
    """
    return point2[2]-point1[2]

class Drone():
    def __init__(self):
        """
        self.positionX/Y/Z := tableau qui enregistre l'historique des positions du drone comme avec un système de motion capture
        Ces tableaux seront utilisés pour animer un point simulant le drone dans l'espace.
        NB: l'unité de distance est le centimère et l'unité de temps est la seconde. En une seconde, nous prendrons 100 frames.
        """
        self.positionX = [0]
        self.positionY = [0]
        self.positionZ = [0]
        
        """
        les vitesses maximales de translation et de rotation ainsi que le pas minimal d'avancé sont pour le moment fixés arbitrairement!
        """
        self.maxSpeed = 500 #cm/s
        self.maxSpeedRotation = 2*math.pi/100 
        self.pasminimal=5
        
        """
        self.gonetoX/Y/Z := tableau qui enregistre les positions des extremites des petits segments décrits par le drone.
        Ces tableau seront plutot utilisé pour visualiser la trajectoire globale décrite par le drone.
        """
        self.goneToX = []
        self.goneToY = []
        self.goneToZ = []
        
        self.alpha = 0 # Dans un système de coordonées cylindrique, il s'agit de l'angle entre le repère du drone mobile et un repère fixe.
        self.fidelity = 0 #Il s'agit d'un compteur de points parcours par le drone dans le cas ou on imposera une suite de points à suivre
        
        self.planning = "" #chaine de caractère qui enregistre le planning de vol en termes de commandes
        
    def takeoff(self):
        """
        décoller
        """
        for i in range(100):#decollage en une seconde soit 100 frames
            self.positionZ += [self.positionZ[-1]+1] # Juste l'altitude est modifiée
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
        self.goneToX += [self.positionX[-1]]
        self.goneToY += [self.positionY[-1]]
        self.goneToZ += [self.positionZ[-1]]

    def land(self):
        """
        attérir
        """
        speed=0.8
        speedUsed=speed*self.maxSpeed
        duration = int(abs(self.positionZ[-1])/speedUsed*10000)/10000
        for i in range (int(duration*100)): #la durée étant en seconde nous aurons le produit par 100 pour les frames
            self.positionZ += [self.positionZ[-1] - speedUsed/100]# Juste l'altitude est modifiée
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.stop(); \n this.land(); \n });"

    def up(self, speed, duration):
        """
        Monter
        """
        speedUsed = speed * self.maxSpeed #speed est un reel entre 0 et 1 qui précise le pourcentage de vitesse à utiliser
        for i in range (int(duration*100)): #la durée étant en seconde nous aurons le produit par 100 pour les frames
            self.positionZ += [self.positionZ[-1] + speedUsed/100]# Juste l'altitude est modifiée
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.up("+str(speed)+"); \n }) \n "

    def down(self, speed, duration):
        """
        Descendre
        """
        print("duration dor down", duration)
        speedUsed = speed * self.maxSpeed
        for i in range(duration * 100):
            self.positionZ += [self.positionZ[-1] - speedUsed / 100]# Juste l'altitude est modifiée
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
            if self.positionZ[-1] < 0: self.positionZ[-1] = 0  # 0 correspond on sol, on peut pas descendre plus bas
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.down("+str(speed)+"); \n }) \n "
        
    def clockwise(self, speed, duration):
        """
        Rotation droite
        """
        print("doing clockwise")
        speedUsed = speed * self.maxSpeedRotation
        print("duration clockwise", duration)
        for i in range(int(duration * 100)):
            self.positionZ += [self.positionZ[-1]]
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
            self.alpha -= speedUsed/100            #il y a juste mise à jour de l'angle. Le sens horaire est négatif.
        print("alpha set to", self.alpha)
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.clockwise("+str(speed)+"); \n }) \n "

    def counterClockwise(self, speed, duration):
        """
        Rotation gauche
        """
        print("doing counterclockwise")
        speedUsed = speed * self.maxSpeedRotation
        for i in range(int(duration * 100)):
            self.positionZ += [self.positionZ[-1]]
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1]]
            self.alpha += speedUsed / 100        #il y a juste mise à jour de l'angle. Le sens horaire est négatif.
        print("alpha set to", self.alpha)
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.counterClockwise("+str(speed)+"); \n }) \n "

    def front(self, speed, duration):
        """
        Avancer 
        """
        print("doing front")
        speedUsed = speed * self.maxSpeed
        for i in range(int(duration * 10000)):
            #il faut bien garder en tete que le drone avance suivant son repère relatif propre (qui est mobile)
            #D'ou une modification suivant les axes X et Y en fonction de l'angle alpha
            self.positionX += [self.positionX[-1] + speedUsed*math.cos(self.alpha) / 10000]
            self.positionY += [self.positionY[-1] + speedUsed*math.sin(self.alpha) / 10000]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speed)+"); \n }) \n "

    def back(self, speed, duration):
        """
        reculer 
        """
        speedUsed = speed * self.maxSpeed
        for i in range(duration * 100):
            #il faut bien garder en tete que le drone avance suivant son repère relatif propre (qui est mobile)
            #D'ou une modification suivant les axes X et Y en fonction de l'angle alpha
            self.positionX += [self.positionX[-1] - speedUsed*math.cos(self.alpha) / 100]
            self.positionY += [self.positionY[-1] - speedUsed*math.sin(self.alpha) / 100]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.back("+str(speed)+"); \n }) \n "

    def left(self, speed, duration):
        """
        translation gauche 
        """
        speedUsed = speed * self.maxSpeed
        for i in range(duration * 100):
            #il faut bien garder en tete que le drone avance suivant son repère relatif propre (qui est mobile)
            #D'ou une modification suivant les axes X et Y en fonction de l'angle alpha
            self.positionX += [self.positionX[-1] - speedUsed*math.sin(self.alpha) / 100]
            self.positionY += [self.positionY[-1] + speedUsed*math.cos(self.alpha) / 100]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.left("+str(speed)+"); \n }) \n "

    def right(self, speed, duration):
        """
        translation droite 
        """
        speedUsed = speed * self.maxSpeed
        for i in range(duration * 100):
            #il faut bien garder en tete que le drone avance suivant son repère relatif propre (qui est mobile)
            #D'ou une modification suivant les axes X et Y en fonction de l'angle alpha
            self.positionX += [self.positionX[-1] + speedUsed*math.sin(self.alpha) / 100]
            self.positionY += [self.positionY[-1] - speedUsed*math.cos(self.alpha) / 100]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.right("+str(speed)+"); \n }) \n "

    def frontClockwise(self, speedFront, speedClockwise, duration):
        """
        avancer + tourner à droite 
        """
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedClockwise = speedClockwise * self.maxSpeedRotation
        for i in range(duration * 100):
            self.alpha -= speedUsedClockwise / 100
            self.positionX += [self.positionX[-1] + speedUsedFront * math.cos(self.alpha) / 100]
            self.positionY += [self.positionY[-1] + speedUsedFront * math.sin(self.alpha) / 100]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+"); \n this.clockwise("+str(speedClockwise)+"); \n }) \n "

    def frontCounterClockwise(self, speedFront, speedCounterClockwise, duration):
        """
        avancer + tourner à gauche 
        """
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedCounterClockwise = speedCounterClockwise * self.maxSpeedRotation
    
        for i in range(duration * 100):
            self.alpha += speedUsedCounterClockwise / 100
            self.positionX += [self.positionX[-1] + speedUsedFront * math.cos(self.alpha) / 100]
            self.positionY += [self.positionY[-1] + speedUsedFront * math.sin(self.alpha) / 100]
            self.positionZ += [self.positionZ[-1]]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+"); \n this.counterClockwise("+str(speedCounterClockwise)+"); \n }) \n "

    def frontUp(self,speedUp,rate,niveau):
        """
        avancer + monter
        """
        print('doing frontup')
        speedUsedUp = speedUp * self.maxSpeed
        speedUsedFront = speedUsedUp*rate #rate=distance dans le XY divisé par la hauteur
        print("speedup", speedUsedUp)
        print('speedfront', speedUsedFront)
        duration = int(10000*niveau/math.sqrt(speedUsedUp**2 + speedUsedFront**2))/10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] + speedUsedFront * math.cos(self.alpha) / 10000.]
            self.positionY += [self.positionY[-1] + speedUsedFront * math.sin(self.alpha) / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedUp*rate)+"); \n this.up("+str(speedUp)+"); \n }) \n "

    def frontDown(self,speedDown,rate,niveau):
        """
        avancer + descendre 
        """
        print('doing frontdown')
        speedUsedDown = speedDown * self.maxSpeed
        speedUsedFront = speedUsedDown*rate #rate=distance XY/hauteur
        duration = int(-10000*niveau/math.sqrt(speedUsedDown**2 + speedUsedFront**2))/10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] - speedUsedFront * math.cos(self.alpha) / 10000.]
            self.positionY += [self.positionY[-1] - speedUsedFront * math.sin(self.alpha) / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedDown*rate)+"); \n this.up("+str(speedDown)+"); \n }) \n "
            
    def leftUp(self,speedLeft, rapportyz, duration):
        """
        translation gauche + monter
        """
        print('doing leftUp')
        speedUsedLeft = speedLeft*self.maxSpeed
        speedUsedUp = speedUsedLeft*rapportyz
        duration = int(10000 * duration / speedUsedLeft) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]  
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.left("+str(speedLeft)+"); \n this.up("+str(speedLeft*rapportyz)+"); \n }) \n "
        
    def leftDown(self,speedLeft, rapportyz, duration):
        """
        translation gauche + descendre
        """
        print('doing leftDown')
        speedUsedLeft = speedLeft*self.maxSpeed
        speedUsedDown = speedUsedLeft*rapportyz
        duration = int(10000 * duration / speedUsedLeft) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.] 
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.left("+str(speedLeft)+"); \n this.down("+str(speedLeft*rapportyz)+"); \n }) \n "


        
    def rightUp(self,speedRight, rapportyz, duration):
        """
        translation droite + monter
        """
        print('doing rightUp')
        speedUsedRight = speedRight*self.maxSpeed
        speedUsedUp = speedUsedRight*rapportyz
        duration = int(10000 * duration / speedUsedRight) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1] - speedUsedRight  / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]  
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.right("+str(speedRight)+"); \n this.up("+str(speedRight*rapportyz)+"); \n }) \n "


            
    def rightDown(self,speedRight, rapportyz, duration):
        """
        translation droite + descendre
        """
        print('doing rightdown')
        speedUsedRight = speedRight*self.maxSpeed
        speedUsedBack = speedUsedRight*rapportyz
        duration = int(10000 * duration / speedUsedRight) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1]]
            self.positionY += [self.positionY[-1] - speedUsedRight  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedBack/10000.] 
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.right("+str(speedRight)+"); \n this.down("+str(speedRight*rapportyz)+"); \n }) \n "

        
    def frontLeftUp(self,speedFront, rapportyx, rapportzx, duration):
        """
        avancer + translation gauche + monter
        """
        print('doing frontLeftUp')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedLeft = speedUsedFront*rapportyx
        speedUsedUp = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] + speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+");\n this.left("+str(speedFront*rapportyx)+"); \n this.up("+str(speedFront*rapportzx)+"); \n }) \n "


    def BackLeftUp(self,speedFront, rapportyx, rapportzx, duration):
        """
        reculer + translation gauche + monter
        """
        print('doing BackLeftUp')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedLeft = speedUsedFront*rapportyx
        speedUsedUp = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] - speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.back("+str(speedFront)+");\n this.left("+str(speedFront*rapportyx)+"); \n this.up("+str(speedFront*rapportzx)+"); \n }) \n "

            
    def frontLeftDown(self,speedFront, rapportyx, rapportzx, duration):
        """
        avancer + translation gauche + descendre
        """
        print('doing frontLeftDown')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedLeft = speedUsedFront*rapportyx
        speedUsedDown = speedUsedFront*rapportzx
        duration = int(100000 * duration / speedUsedFront) / 100000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] + speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+");\n this.left("+str(speedFront*rapportyx)+"); \n this.down("+str(speedFront*rapportzx)+"); \n }) \n "



    def BackLeftDown(self,speedFront, rapportyx, rapportzx, duration):
        """
        reculer + translation gauche + descendre
        """
        print('doing BackLeftDown')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedLeft = speedUsedFront*rapportyx
        speedUsedDown = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] - speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] + speedUsedLeft  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.back("+str(speedFront)+");\n this.left("+str(speedFront*rapportyx)+"); \n this.down("+str(speedFront*rapportzx)+"); \n }) \n "



    def frontRightUp(self,speedFront, rapportyx, rapportzx, duration):
        """
        avancer + translation droite + monter
        """
        print('doing frontRightUp')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedRight = speedUsedFront*rapportyx
        speedUsedUp = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] + speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] - speedUsedRight  / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+");\n this.right("+str(speedFront*rapportyx)+"); \n this.up("+str(speedFront*rapportzx)+"); \n }) \n "


            
    def BackRightUp(self, speedFront, rapportyx, rapportzx, duration):
        """
        reculer + translation droite + monter
        """
        print('doing BackRightUp')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedRight = speedUsedFront * rapportyx
        speedUsedUp = speedUsedFront * rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] - speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] - speedUsedRight / 10000.]
            self.positionZ += [self.positionZ[-1] + speedUsedUp / 10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.back("+str(speedFront)+");\n this.right("+str(speedFront*rapportyx)+"); \n this.up("+str(speedFront*rapportzx)+"); \n }) \n "



    def frontRightDown(self,speedFront, rapportyx, rapportzx, duration):
        """
        avancer + translation droite + descendre
        """
        print('doing frontRightDown')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedRight = speedUsedFront*rapportyx
        speedUsedDown = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] + speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] - speedUsedRight  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.front("+str(speedFront)+");\n this.right("+str(speedFront*rapportyx)+"); \n this.down("+str(speedFront*rapportzx)+"); \n }) \n "



    def BackRightDown(self,speedFront, rapportyx, rapportzx, duration):
        """
        reculer + translation droite + descendre
        """
        print('doing BackRightDown')
        speedUsedFront = speedFront * self.maxSpeed
        speedUsedRight = speedUsedFront*rapportyx
        speedUsedDown = speedUsedFront*rapportzx
        duration = int(10000 * duration / speedUsedFront) / 10000.
        print("duration", duration)
        for i in range(int(duration * 10000)):
            self.positionX += [self.positionX[-1] - speedUsedFront / 10000.]
            self.positionY += [self.positionY[-1] - speedUsedRight  / 10000.]
            self.positionZ += [self.positionZ[-1] - speedUsedDown/10000.]
        self.planning+= ".after("+ str(duration*1000)+ ", function() { \n this.back("+str(speedFront)+");\n this.right("+str(speedFront*rapportyx)+"); \n this.down("+str(speedFront*rapportzx)+"); \n }) \n "



    def goToPoint(self,point, pas):
        """
        Planification du parcours point par point en mode "avion". En fait le drone se tourne toujours vers son point cible 
        avant d'avancer vers celui ci.
        """
        point1 = [self.positionX[-1], self.positionY[-1], self.positionZ[-1]] #position actuelle du drone
        if distanceXY(point, point1) >= pas and denivellation(point,point1)**2 >= pas:
            print("going to point")
            theta = math.atan2(point[1],point[0]) #angle du point à atteindre
            phi = math.atan2(point1[1], point1[0]) #angle absolu du drone en tant que point
            print("theta",theta)
            #angle := angle relatif selon lequel le drone doit se tourner pour faire face au point cible
            angle = math.atan2(distanceXY(point,[0,0])*math.sin(theta-self.alpha)-distanceXY(point1,[0,0])*math.sin(phi-self.alpha), distanceXY(point,[0,0])*math.cos(theta-self.alpha)-distanceXY(point1,[0,0])*math.cos(phi-self.alpha))
            print("angle", angle)
            print(int((angle/self.maxSpeedRotation)*10000)/10000.0)
            if angle < 0:
                self.clockwise(1,int(-(angle/self.maxSpeedRotation)*10000)/10000.0)

            else :
                self.counterClockwise(1,int((angle/self.maxSpeedRotation)*10000)/10000.0)
            if denivellation(point1,point)==0:
                rate=0
            else:
                rate = distanceXY(point1, point)/denivellation(point1, point) 
            print("rate",rate)
            niveau = denivellation(point1, point)
            if rate > 0:
                self.frontUp(1,rate, niveau)
            elif rate< 0:
                self.frontDown(1, rate, niveau)
            else:
                self.front(1, int(((distanceXY(point1, point) /self.maxSpeed)*10000)/10000.0))
            self.fidelity += 1
            
        else: 
            print("skipping point")
        self.goneToX += [self.positionX[-1]]
        self.goneToY += [self.positionY[-1]]
        self.goneToZ += [self.positionZ[-1]]

    def goToPoint1(self, point, pas):
        """
        Planificateur de parcours point à point en utilisant uniquement des translations et pas de rotation.
        """
        point1 = [self.positionX[-1], self.positionY[-1], self.positionZ[-1]]
        if distance(point1,point) > pas:
            if (point[0]-point1[0]) > 0.2 or (point[0]-point1[0]) < -0.2:
                print("going to point")
                rapportyx= (point[1]-point1[1])/(point[0]-point1[0])
                rapportzx = (point[2] - point1[2]) / (point[0] - point1[0])
        

                if (point[0]-point1[0])> 0:
                    if (point[1]-point1[1])>0:
                        if (point[2] - point1[2]) >0:
                            self.frontLeftUp(1, rapportyx, rapportzx, (point[0]-point1[0]) )
                        else:
                            self.frontLeftDown(1, rapportyx, -rapportzx, (point[0] - point1[0]))
                    else:
                        if (point[2] - point1[2]) >0:
                            self.frontRightUp(1, -rapportyx, rapportzx, (point[0]-point1[0]) )
                        else:
                            self.frontRightDown(1, -rapportyx, -rapportzx, (point[0] - point1[0]))
                else:
                    if (point[1]-point1[1])>0:
                        if (point[2] - point1[2]) >0:
                            self.BackLeftUp(1, -rapportyx, -rapportzx, -(point[0]-point1[0]) )
                        else:
                            self.BackLeftDown(1, rapportyx, rapportzx, -(point[0] - point1[0]))
                    else:
                        if (point[2] - point1[2]) >0:
                            self.BackRightUp(1, rapportyx, -rapportzx, -(point[0]-point1[0]) )
                        else:
                            self.BackRightDown(1, rapportyx, rapportzx, -(point[0] - point1[0]))
            else:
                if (point[1]-point1[1])>0:
                    if (point[2] - point1[2]) >0:
                        self.leftUp(1, (point[2] - point1[2]) / (point[1] - point1[1]), (point[1]-point1[1]) )
                    else:
                        self.leftDown(1,-(point[2] - point1[2]) / (point[1] - point1[1]), (point[1] - point1[1]))
                elif (point[1]-point1[1])<0:
                    if (point[2] - point1[2]) >0:
                        self.rightUp(1, -(point[2] - point1[2]) / (point[1] - point1[1]), -(point[1]-point1[1]) )
                    else:
                        self.rightDown(1, (point[2] - point1[2]) / (point[1] - point1[1]), -(point[1] - point1[1]))
                else:
                    if (point[2] - point1[2]) >0:
                        self.up(1, int((((point[2] - point1[2])/self.maxSpeed)*10000)/10000.0))
                    else:
                        self.down(1, -int((((point[2] - point1[2])/self.maxSpeed)*10000)/10000.0))

            self.fidelity += 1
        else:
            print("skipping point")
        self.goneToX += [self.positionX[-1]]
        self.goneToY += [self.positionY[-1]]
        self.goneToZ += [self.positionZ[-1]]