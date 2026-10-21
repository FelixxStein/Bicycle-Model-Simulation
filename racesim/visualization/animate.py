import pygame 
import numpy as np
class Animate:
    def __init__(self, path="runs/seed_1900.npz",
                 window_width=1200,
                window_height=785,
                scale_factor=6,
                Look_ahead_car=1,
                smoothing_factor_zoom=0.01,
                smoothing_factor_center=0.05,
                upscale_car=1, 
                min_zoom=0.4,
                max_zoom=20,
                margin=0.75):
        self.margin=margin
        self.min_zoom=min_zoom
        self.max_zoom=max_zoom
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
        self.smoothing_factor_zoom=smoothing_factor_zoom
        self.smoothing_factor_center=smoothing_factor_center
        self.last_sprite_zoom=0
        
    
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
        raw_sprite=pygame.image.load("assets/racecar.png")
        sprite_scale=(self.wheelbase*self.scale_factor)/121
        raw_sprite=raw_sprite.convert_alpha()
        w,h=raw_sprite.get_size()
        scaled_sprite=pygame.transform.smoothscale(raw_sprite, (int(200), int(200*h/w)))
        sw,sh=scaled_sprite.get_size()
        car_sprite=pygame.transform.smoothscale(scaled_sprite, (int(sw*sprite_scale*self.upscale_car), int(sh*sprite_scale*self.upscale_car)))
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
                        sprite_scale=(self.wheelbase*self.scale_factor)/121
                        car_sprite=pygame.transform.smoothscale(scaled_sprite, (int(sw*sprite_scale*self.upscale_car), int(sh*sprite_scale*self.upscale_car)))
                    elif event.key == pygame.K_PLUS:
                        self.scale_factor=self.scale_factor+0.2
                        sprite_scale=(self.wheelbase*self.scale_factor)/121
                        car_sprite=pygame.transform.smoothscale(scaled_sprite, (int(sw*sprite_scale*self.upscale_car), int(sh*sprite_scale*self.upscale_car)))
                    elif event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        i=0
            screen.fill((0, 100, 0))
            k = min(len(self.path_trajectory_array[...,0,0])-1,i // self.substeps)
            #camera
            point_camera=np.concatenate((self.cartrace[None,i],self.current_trajectory[k]),axis=0)
            min_point_camera=np.min(point_camera,axis=0)
            max_point_camera=np.max(point_camera,axis=0)
            box_size=np.maximum(20,max_point_camera-min_point_camera)
            target_center=(max_point_camera+min_point_camera)/2
            target_zoom=np.minimum(self.window_width/box_size[0],self.window_height/box_size[1])*self.margin
            target_zoom=np.clip(target_zoom,self.min_zoom,self.max_zoom)
            self.scale_factor = self.scale_factor+self.smoothing_factor_zoom*(target_zoom-self.scale_factor)
            self.camera_center= self.camera_center+self.smoothing_factor_center*(target_center-self.camera_center)
            current_border_right=self.world_to_screen(self.borderright,self.camera_center)
            current_border_left=self.world_to_screen(self.borderleft,self.camera_center)
            if abs(self.scale_factor - self.last_sprite_zoom) > 0.02 * abs(self.last_sprite_zoom):
                sprite_scale=(self.wheelbase*self.scale_factor)/121
                car_sprite=pygame.transform.smoothscale(scaled_sprite, (int(sw*sprite_scale*self.upscale_car), int(sh*sprite_scale*self.upscale_car)))
                self.last_sprite_zoom=self.scale_factor
            #draw border
            pygame.draw.polygon(screen,color=(55, 55, 55),points=current_border_left)
            pygame.draw.polygon(screen,color=(0, 100, 0),points=current_border_right)
            pygame.draw.polygon(screen,color="White",points=current_border_left, width=3)
            pygame.draw.polygon(screen,color="White",points=current_border_right,width=3)
            #draw MPPI lines
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
    animation=Animate()
    animation.main_loop()
