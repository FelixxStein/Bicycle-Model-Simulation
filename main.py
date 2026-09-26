import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
import csv
import pandas as pd
import pygame 
from pygame.locals import *


import matplotlib.pyplot as plt
import csv



window_widht=1200
window_height=785
padding=200
xpos=[]
ypos=[]
df = pd.read_csv("/Users/felixarbeit/Documents/Strategiewechsel/Bicycle-Model-Simulation/Hockenheim.csv")
x=df.iloc[:, 0].tolist()
y=df.iloc[:, 1].tolist()
x=np.array(x)
y=np.array(y)
points = np.column_stack((x, y))








car=vehicle.vehicle(starting_x=x[0],starting_y=y[0] ,velocity=15,starting_steering_angle=30)
for i in range(1000):
    car.update_position()
    xpos.append(car.x)
    ypos.append(car.y)




xpos=xpos+abs(min(x))
ypos=ypos+abs(min(y))

x=x+abs(min(x))
y=y+abs(min(y))

scale_factor=5
xpos=xpos*scale_factor
ypos=ypos*scale_factor
x=x*scale_factor
y=y*scale_factor
ypos=abs(ypos-max(y))
y=abs(y-max(y))
carpos = np.column_stack((xpos, ypos))

offsetx=(-carpos[0][0])+window_widht/2
offsety=(-carpos[0][1])+window_height/2

pygame.init()
screen = pygame.display.set_mode((window_widht, window_height))
screen.fill((255, 255, 255))



clock = pygame.time.Clock()
running = True
currentcarpoint=carpos[0]
xoffset=currentcarpoint
yoffset=currentcarpoint
i=0
while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    screen.fill((255, 255, 255))
    # fill the screen with a color to wipe away anything from last frame
    
    pygame.draw.circle(screen, "red", currentcarpoint, 1)
    
    # RENDER YOUR GAME HERE
    currentcarpoint=np.array(carpos[i])
    
    offsetx=(window_widht/2)-(carpos[i][0])
    offsety=(window_height/2)-(carpos[i][1])
    shiftedpoints=np.rint(points+np.array([offsetx,offsety])).astype(int)
    if i< len(carpos)-1:
        i=i+1
        
    pygame.draw.polygon(screen, "BLUE",shiftedpoints,width=3)
        
    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()




