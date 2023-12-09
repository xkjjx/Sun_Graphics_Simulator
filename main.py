import pygame
import numpy as np
from numpy import sin,cos,tan,arcsin,arccos,arctan,pi
from math import sqrt
import sys
import os
import time

GLOBAL_SUN_LOCATION = np.array([0,0,0])
earthLocation = np.array([1,1,0])
TIMESCALE = 5 #represents days per second

class Sun():
    def __init__(self, location, diameter=None):
        self.location = location
        #diamter could be used later to get a realistic size of the Sun
        self.diameter = diameter

class Planet():
    def __init__(self):
        pass

    def setLocationByCartesian(self,location):
        self.location = location
        self.angle = arctan(location[1]/location[0])
        self.radius = np.linalg.norm(location)


    def setLocationByPolar(self,axis,radius,startingAngle=0):
        self.axis = axis
        self.radius = radius
        self.angle = startingAngle
        self.location = np.array([radius*cos(startingAngle),radius*sin(startingAngle),0])
        #could be used later to make simulation the revolution around the sun easier

    def revolve(self,angularDistance):
        self.angle += angularDistance
        self.location = np.array([self.radius*cos(self.angle),self.radius*sin(self.angle),0])




sun = Sun(GLOBAL_SUN_LOCATION)
earth = Planet()
earth.setLocationByCartesian(earthLocation)


def getRotationMatrixFunction(axis):
    def retFunction(theta):
        u_x, u_y, u_z = axis  # assuming axis is a tuple or list with three components
        R = np.array([[cos(theta) + u_x**2 * (1 - cos(theta)), u_x * u_y * (1 - cos(theta)) - u_z * sin(theta), u_x * u_z * (1 - cos(theta)) + u_y * sin(theta)],
                      [u_y * u_x * (1 - cos(theta)) + u_z * sin(theta), cos(theta) + u_y**2 * (1 - cos(theta)), u_y * u_z * (1 - cos(theta)) - u_x * sin(theta)],
                      [u_z * u_x * (1 - cos(theta)) - u_y * sin(theta), u_z * u_y * (1 - cos(theta)) + u_x * sin(theta), cos(theta) + u_z**2 * (1 - cos(theta))]])
        return R
    return retFunction
def vectorProject(u,v):
    return np.dot(u,v)/(np.linalg.norm(v)**2) * v

def planeProject(u,p):
    return u - vectorProject(u,p)

def unitizeVector(a):
    return a/np.linalg.norm(a)


class POV():
    def __init__(self,object,eigenMatrix):
        self.object = object
        self.eigenMatrix = eigenMatrix

    def rotate(self,axis,angularDistance):
        rotFunc = getRotationMatrixFunction(axis)
        #self.eigenMatrix = np.linalg.inv(rotFunc(pi/180) @ np.linalg.inv(self.eigenMatrix))
        self.eigenMatrix @= rotFunc(angularDistance)


    def getCoordinates(self):
        #change of basis to use our basis vectors
        #project new vectors to spherical coordinates
        #project spherical coordinates back to plane
        loc = sun.location - self.object.location
        #print("Relative location:",loc)
        #print("Eigenmatrix:",self.eigenMatrix)
        # print(self.eigenMatrix)
        #print(np.linalg.inv(self.eigenMatrix))
        locNewBasis = np.linalg.inv(np.transpose(self.eigenMatrix)) @ loc
        #print("Relative location in new coordinates:",locNewBasis)
        r = np.linalg.norm(locNewBasis)
        
        x,y,z = locNewBasis[0],locNewBasis[1],locNewBasis[2]

        if z > 0:
            phi = arctan(np.linalg.norm(locNewBasis[:-1])/z)
        elif z < 0:
            phi = arctan(np.linalg.norm(locNewBasis[:-1])/z) + pi
        else:
            phi = pi/2

        if x > 0:
            theta = arctan(y/x)
        elif x < 0:
            if y >= 0:
                theta = arctan(y/x) + pi
            else:
                theta  =arctan(y/x) - pi
        else:
            if y >= 0:
                theta = pi/2
            else:
                theta = -pi/2

        #print("ANGLES: ",theta*180/pi,phi*180/pi)
        ret = np.array([cos(theta)*sin(phi),cos(phi)])
        return ret,(sin(theta) > 0)





        #sun_x,sun_y = planeProject(loc - np.array([self.x,self.y,self.z]),np.array([self.x,self.y,self.z]))
        # sun_x,sun_y = renderSun(self.object, [self.x,self.y,self.z])
        # sun_x *= xMid
        # sun_y *= yMid
        # sun_x += xMid
        # sun_y += yMid
        # return [sun_x,sun_y]


view = POV(earth,np.array([[-1.0,0,0],
                           [0,0,1.0],
                           [0,1.0,0]]))
# test = np.array([[1.0,1.0,0.0],
#                  [0.0,1.0,0.0]])
# rotFunc = getRotationMatrixFunction([0,0,1])
#
# for i in range(181):
#     print(i)
#     print(test)
#     test @= rotFunc(pi/180)




# Initialize Pygame
pygame.init()

# Set up display
width, height = 800, 600

screen = pygame.display.set_mode((width - 100, height-100))
pygame.display.set_caption("My Animation")

# Set up colors
black = (50, 170, 220)
white = (255, 255, 255)
x = 0
# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Clear the screen
    screen.fill(black)

    rotationAxis = np.array([0,0.4,1])
    x += 1
    if x % 24 == 0:
        #time.sleep(10000)
        print("Day ",x//24)
        print("Location:",earth.location)
    if x % (366*24+1) == 0:
        print("About year",x//366)
    view.rotate(unitizeVector(rotationAxis),TIMESCALE/24)
    earth.revolve(TIMESCALE/(366.24*24))
    coords,valid = view.getCoordinates()
    coords = coords * np.array([width//2,height//2]) + np.array([(width-100)//2,(height-100)//2])
    sun_x,sun_y = coords
    # Draw the rectangle
    if valid and abs(coords[0]) < width - 100 and abs(coords[1]) < height - 100:
        pygame.draw.circle(screen, white, (sun_x, sun_y),50)

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(60)







