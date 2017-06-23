#-*- coding: utf-8 -*-
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np


def tracerTrajectoire(X,Y,Z,X1,Y1,Z1):
    fig1 = plt.figure(1)
    ax = fig1.gca(projection='3d')

    trajectoire2, = ax.plot(X, Y, Z ,'-' ,color="red", label='trajectoire du drone')
    trajectoireTheorique, = ax.plot(X1, Y1, Z1, 'o', label='points a atteindre theorique')
    trajectoire3, = ax.plot(X, Y, Z ,'x' ,label='trajectoire du drone')


    #ax.set_xlim([100, 1000])
    #ax.set_ylim([100, 1000])
    #ax.set_zlim([100, 1000])
    ax.set_title('Trajectoires')
    ax.legend()
    plt.show()


# animation du mobile sur sa trajectoire
def update_point(n, X1 ,Y1 ,Z1 ,point, trajet):
    point.set_data(np.array([X1[n], Y1[n]]))
    point.set_3d_properties(Z1[n], 'z')
    trajet.set_data(np.array([X1[:n], Y1[:n]]))
    trajet.set_3d_properties(Z1[:n], 'z')
    return point, trajet


def animerPoints(X, Y, Z):
    fig2 = plt.figure(2)
    ax = p3.Axes3D(fig2)

    mobile, = ax.plot([X[0]], [Y[0]], [Z[0]], 'x', label='drone')
    trajectoire, = ax.plot([X[0]], [Y[0]], [Z[0]], '-', color="red", label='trajectoire du drone')
    ani=animation.FuncAnimation(fig2, update_point, len(X), fargs=(X, Y, Z, mobile, trajectoire),interval=10, blit = False)

    ax.set_xlim3d([min(X), max(X)])
    ax.set_xlabel('X')
    ax.set_ylim3d([min(Y), max(Y)])
    ax.set_ylabel('Y')
    ax.set_zlim3d([min(Z), max(Z)])
    ax.set_zlabel('Z')
    ax.set_title('Animation drone')
    ax.legend()
    plt.show()

