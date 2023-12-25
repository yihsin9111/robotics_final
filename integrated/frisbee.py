from math import pi
import math
import matplotlib.pyplot as plt
import numpy as np
import copy

min_deg = 6.5
max_deg = 26
dtheta = (26-6.5)/4
deg_list = [min_deg, min_deg+dtheta, min_deg+dtheta*2, min_deg+dtheta*3, min_deg+dtheta*4]
frisbee_v = 11.78
init_height = 0.0

starting_z = [2.8, 5.3, 6.5]

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

        self.init_z = 0
        self.deg = 0
        self.targy = 0

    def simulate(self, y0, vx0, alpha, deg, dt):
        # lift coefficient
        cl = self.CLO + self.CLA*alpha/180

        # drag coefficient
        cd = self.CDO + self.CDA*(((alpha-self.ALPHAO)*pi/180)**2)

        self.x = 0
        self.y = y0
        self.vx = vx0*math.cos(math.radians(deg))
        self.vy = vx0*math.sin(math.radians(deg))
        self.x_list.clear()
        self.y_list.clear()

        while self.x < self.init_z :
            dy = (self.RHO*(self.vx**2)*self.AREA*cl/2/self.m+self.g)*dt
            dx = -self.RHO*(self.vx**2)*self.AREA*cd*dt
            
            self.vx = self.vx + dx
            self.vy = self.vy + dy
            self.x = self.x + self.vx*dt
            self.y = self.y + self.vy*dt
            self.x_list.append(self.x)
            self.y_list.append(self.y)
            self.vx_list.append(self.vx)
    
    def get_z(self, y):
        if(y >= 1.62):
            return -1
        y = y - 0.34 - 0.14
        self.targy = y
        print("Self.targy:", self.targy)
        if(y <= 0.13 and y > -0.12):
            self.init_z = starting_z[0]
            self.deg = min_deg
            print("Hi")
        elif(y <= -0.12 and y > -0.5):
            self.init_z = starting_z[1]
            self.deg = min_deg
            print("Hi1")
        elif(y < -0.5):
            self.init_z = starting_z[2]
            self.deg = min_deg
            print("Hi2")
        elif(y >= 1.2):
            self.init_z = 2.5
            self.deg = max_deg
        elif(y >= 0.75):
            self.init_z = 2
            self.deg = deg_list[3]
        elif(y > 0.38):
            self.init_z = 4
            self.deg = deg_list[2]
        elif(y>0.13):
            self.init_z = 2.5
            self.deg = deg_list[1]
        print("self.deg:", self.deg)
        return_z = self.init_z
        last_delta_y = 100
        delta_y = 100
        while(not ((delta_y) < 0.05 and last_delta_y >= delta_y)):
            return_z = self.init_z
            self.simulate(init_height+0.5*math.sin(math.radians(self.deg)), frisbee_v, 0, self.deg, 0.01)
            last_delta_y = delta_y
            delta_y = abs(self.y - self.targy)
            self.init_z += 0.05
        self.init_z = return_z
        return return_z

    def get_angle(self):
        return self.deg

    def show_plt(self):
        self.simulate(init_height+0.5*math.sin(math.radians(self.deg)), frisbee_v, 0, self.deg, 0.01)
        plt.plot(self.x_list, self.y_list, 'b*')
        plt.show()
    

# fris = Frisbee()
# print(fris.get_z(1.6))
# print(fris.get_angle())
# fris.show_plt()


