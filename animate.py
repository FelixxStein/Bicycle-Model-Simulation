from pygame.locals import *
import pygame 
import numpy as np
class animate:
    def __init__(self, path="runs/seed_1900.npz",
                 window_width=1200,
                window_height=785,
                scale_factor=6,
                Look_ahead_car=1,
                smoothing_factor=0.075,
                upscale_car=1):
        self.data=np.load(path)
        self.path_trajectory_array=self.data["path_trajectory_array"]
        self.weights_array=self.data["weights_array"]
        self.off_track_array=self.data["off_track_array"]
        self.current_trajectory=self.data["current_trajectory"]
        self.substeps=self.data["substeps"]
        self.upscale_car=upscale_car
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
        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale*self.upscale_car), int(h*sprite_scale*self.upscale_car)))
        running = True
        i=0
        transp=pygame.Surface((self.window_width, self.window_height), pygame.SRCALPHA)
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_MINUS:
                        self.scale_factor=self.scale_factor-0.2
                        sprite_scale=(self.wheelbase*self.scale_factor)/1060
                        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale*self.upscale_car), int(h*sprite_scale*self.upscale_car)))
                    if event.key == pygame.K_PLUS:
                        self.scale_factor=self.scale_factor+0.2
                        sprite_scale=(self.wheelbase*self.scale_factor)/1060
                        car_sprite=pygame.transform.smoothscale(raw_sprite, (int(w*sprite_scale*self.upscale_car), int(h*sprite_scale*self.upscale_car)))
                    elif event.key == pygame.K_ESCAPE:
                        running = False
            screen.fill((0, 100, 0))
            #camera
            target=self.cartrace[i]+self.Look_ahead_car*self.velocity_vector[i]
            self.camera_center=self.camera_center+self.smoothing_factor*(target-self.camera_center)
            current_border_right=self.world_to_screen(self.borderright,self.camera_center)
            current_border_left=self.world_to_screen(self.borderleft,self.camera_center)
            #draw border
            pygame.draw.polygon(screen,color=(55, 55, 55),points=current_border_left)
            pygame.draw.polygon(screen,color=(0, 100, 0),points=current_border_right)
            pygame.draw.polygon(screen,color="White",points=current_border_left, width=3)
            pygame.draw.polygon(screen,color="White",points=current_border_right,width=3)
            #draw MPPI lines
            k = min(len(self.path_trajectory_array[...,0,0])-1,i // self.substeps)
            cartrace_prepend=self.cartrace[k*self.substeps]
            cartrace_prepend=cartrace_prepend[None, None,...]
            cartrace_prepend=np.repeat(cartrace_prepend,len(self.path_trajectory_array[0,...,0]),axis=0)
            Mppi_lines=np.concatenate((cartrace_prepend,self.path_trajectory_array[k]),axis=1)
            Mppi_lines=self.world_to_screen(Mppi_lines,self.camera_center)
            transp.fill((0, 0, 0, 0))
            for p in range(len(self.path_trajectory_array[0,...,0])-1,-1,-1):
                alpha=max(int(self.weights_array[k][p] * 255),20)
                if self.off_track_array[k][p]==False: line_color=(0, 255, 255, alpha)
                else: line_color=(255, 50, 50, 150)
                pygame.draw.lines(transp, line_color, False, Mppi_lines[p], 2)
            screen.blit(transp, (0, 0))
            cartrace_prepend=self.cartrace[k*self.substeps]
            cartrace_prepend=cartrace_prepend[None,...]
            Optimal_path=np.concatenate((cartrace_prepend,self.current_trajectory[k]),axis=0)
            Optimal_path=self.world_to_screen(Optimal_path,self.camera_center)
            pygame.draw.lines(screen, (255, 215, 0), False, Optimal_path, 3)
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
