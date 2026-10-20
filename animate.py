from pygame.locals import *
import pygame 
import numpy as np
class animate:
    def __init__(self, path="runs/seed_1900.npz",
                 window_width=1200,
                window_height=785,
                scale_factor=4,
                Look_ahead_car=1,
                smoothing_factor=0.075):
        self.data=np.load(path)
        self.Look_ahead_car=Look_ahead_car
        self.wheelbase=self.data["wheelbase"]
        self.borderleft=self.data["border_left"]
        self.borderright=self.data["border_right"]
        self.cartrace=self.data["cartrace"]
        self.dt=self.data["dt"]
        self.window_width=window_width
        self.window_height=window_height
        self.scale_factor=scale_factor
        self.velocity_list=self.data["velocity_list"]
        self.heading_angle=self.data["heading_angle"]
        self.velocity_vector=(self.velocity_list*np.array([np.cos(self.heading_angle),np.sin(self.heading_angle)])).T
        self.camera_center=self.cartrace[0]
        self.smoothing_factor=smoothing_factor
        
    
    def world_to_screen(self,arr,camera_center):
        half_window_size=np.array([self.window_width/2,self.window_height/2])
        array=arr.copy()
        array=array-camera_center
        array=array*self.scale_factor
        array[...,1]=array[...,1]*(-1)
        array=array+half_window_size
        return array

    def main_loop(self):

        pygame.init()
        screen = pygame.display.set_mode((self.window_width, self.window_height))
        screen.fill((255, 255, 255))
        clock = pygame.time.Clock()
        raw_sprite=pygame.image.load("racecar.png")
        sprite_scale=(self.wheelbase*self.scale_factor)/1060
        raw_sprite=raw_sprite.convert_alpha()
        w,h=raw_sprite.get_size()
        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale), int(h*sprite_scale)))
        running = True
        i=0
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_MINUS:
                        self.scale_factor=self.scale_factor-0.2
                        sprite_scale=(self.wheelbase*self.scale_factor)/1060
                        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale), int(h*sprite_scale)))
                    if event.key == pygame.K_PLUS:
                        self.scale_factor=self.scale_factor+0.2
                        sprite_scale=(self.wheelbase*self.scale_factor)/1060
                        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale), int(h*sprite_scale)))
                    elif event.key == pygame.K_ESCAPE:
                        running = False
            screen.fill("Black")
            #camera
            target=self.cartrace[i]+self.Look_ahead_car*self.velocity_vector[i]
            self.camera_center=self.camera_center+self.smoothing_factor*(target-self.camera_center)
            current_border_right=self.world_to_screen(self.borderright,self.camera_center)
            current_border_left=self.world_to_screen(self.borderleft,self.camera_center)
            #draw border
            pygame.draw.polygon(screen,color="Gray",points=current_border_left)
            pygame.draw.polygon(screen,color="Black",points=current_border_right)
            #draw car
            current_angle=np.degrees(self.heading_angle[i])
            rotated_sprite=pygame.transform.rotate(car_sprite, current_angle)
            current_position=self.world_to_screen(self.cartrace[i],self.camera_center)
            screen.blit(rotated_sprite, rotated_sprite.get_rect(center=current_position))

            i=(i+2)%len(self.cartrace)
            pygame.display.flip()
            clock.tick(50)
        pygame.quit()

if __name__ == "__main__":
    animation=animate()
    animation.main_loop()













'''

    # fill the screen with a color to wipe away anything from last frame
    
    pygame.draw.circle(screen, "red", currentcarpoint, 5)
    
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


'''
