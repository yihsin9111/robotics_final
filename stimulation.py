from math import pi
import math
import matplotlib.pyplot as plt
import numpy as np

class Frisbee:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.g = -9.8
        self.m = 0.175
        self.RHO = 1.23
        self.AREA = 0.0572
        self.CLO = 0.1
        self.CLA = 1.4
        self.CDO = 0.08
        self.CDA = 2.72
        self.ALPHAO = -4

        self.x_list = []
        self.y_list = []
        self.vx_list = []

    def simulate(self, y0, vx0, alpha, deg, dt):
        # lift coefficient
        cl = self.CLO + self.CLA*alpha/180

        # drag coefficient
        cd = self.CDO + self.CDA*(((alpha-self.ALPHAO)*pi/180)**2)

        self.x = 0
        self.y = y0
        self.vx = vx0*math.cos(pi*deg/180)
        self.vy = vx0*math.sin(pi*deg/180)

        while self.y>0 :
            dy = (self.RHO*(self.vx**2)*self.AREA*cl/2/self.m+self.g)*dt
            dx = -self.RHO*(self.vx**2)*self.AREA*cd*dt
            
            self.vx = self.vx + dx
            self.vy = self.vy + dy
            self.x = self.x + self.vx*dt
            self.y = self.y + self.vy*dt
            self.x_list.append(self.x)
            self.y_list.append(self.y)
            self.vx_list.append(self.vx)
            print(self.x, " ", self.y)

            print(self)
        
    def show_plt(self):
        ax = plt.figure().add_subplot(projection='3d')

        # Plot a sin curve using the x and y axes.

        # Fixing random state for reproducibility
        # By using zdir='y', the y value of these points is fixed to the zs value 0
        # and the (x, y) points are plotted on the x and z axes.
        ax.scatter(self.x_list, self.y_list, zs=0, zdir='y', label='points in (x, z)')
        ax.bar3d(self.x_list, y = 0.5, z = 0, dx = 0.01, dy = 0.05, dz = self.vx_list)

        # Make legend, set axes limits and labels
        ax.legend()
        ax.set_xlim(0, max(self.x_list))
        ax.set_ylim(0, max(self.y_list))
        ax.set_zlim(0, max(self.vx_list))
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Customize the view angle so it's easier to see that the scatter points lie
        # on the plane y=0
        ax.view_init(elev=20., azim=-35, roll=0)

        plt.show()

fris = Frisbee()
fris.simulate(0.93, 10.78, 10, 7.98, 0.01)
fris.show_plt()