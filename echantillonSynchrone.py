#-*- coding: utf-8 -*-


import math

def distance(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 +  (point1[2]-point2[2])**2)


def alignerEchantillon(pointsTheoriques, pointsMesures) :
    nouvelleSuite=[[],[],[]]
    d=0
    s = distance([pointsMesures[0][0], pointsMesures[1][0], pointsMesures[2][0]],
                 [pointsTheoriques[0][d], pointsTheoriques[1][d], pointsTheoriques[2][d]])
    i=1
    while i < len(pointsMesures[0]) and d < len(pointsTheoriques[0]):

        m=distance([pointsMesures[0][i],pointsMesures[1][i], pointsMesures[2][i]], [pointsTheoriques[0][d], pointsTheoriques[1][d], pointsTheoriques[2][d]])
        if m>=s :
            nouvelleSuite[0]+= [pointsMesures[0][i]]
            nouvelleSuite[1]+= [pointsMesures[1][i]]
            nouvelleSuite[2]+= [pointsMesures[2][i]]
            d+=1
            if d==len(pointsTheoriques[0]): break
            s = distance([pointsMesures[0][i], pointsMesures[1][i], pointsMesures[2][i]],
                         [pointsTheoriques[0][d], pointsTheoriques[1][d], pointsTheoriques[2][d]])
        else:
            s=m
        i+=1
    return nouvelleSuite
