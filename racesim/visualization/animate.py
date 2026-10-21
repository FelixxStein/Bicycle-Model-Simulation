import pygame 
import numpy as np
class Animate:
    def __init__(self, 
                path="runs/seed_1900.npz",
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
        self.has_rollouts=False
        if self.data["controller"]=="mppi" and "path_trajectory_array" in self.data["controller"]:
            self.has_rollouts=True
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
        self.running = True
        raw_sprite=pygame.image.load("assets/racecar.png")
        raw_sprite=raw_sprite.convert_alpha()
        w,h=raw_sprite.get_size()
        self.scaled_sprite=pygame.transform.smoothscale(raw_sprite, (int(200), int(200*h/w)))
        self.sw,self.sh=self.scaled_sprite.get_size()
        self.k=0
        self.i=0
        
    
    def world_to_screen(self,arr,camera_center):
        half_window_size=np.array([self.window_width/2,self.window_height/2])
        array=arr.copy()
        array=array-camera_center
        array=array*self.scale_factor
        array[...,1]=array[...,1]*(-1)
        array=array+half_window_size
        return array
    def setup(self):
        self.setup()
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        self.clock = pygame.time.Clock()
        self.rescale_sprite()

    def rescale_sprite(self):
        sprite_scale=(self.wheelbase*self.scale_factor)/121
        self.car_sprite=pygame.transform.smoothscale(self.scaled_sprite, (int(self.sw*sprite_scale*self.upscale_car), int(self.sh*sprite_scale*self.upscale_car)))
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_MINUS:
                    self.scale_factor=self.scale_factor-0.2
                    self.scale_factor(self.sw,self.sh,self.scaled_sprite)
                elif event.key == pygame.K_PLUS:
                    self.scale_factor=self.scale_factor+0.2
                    self.scale_factor(self.sw,self.sh,self.scaled_sprite)
                elif event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    i=0

    def update_camera(self):
        if self.has_rollouts: 
            point_camera=np.concatenate((self.cartrace[None,self.i],self.current_trajectory[self.k]),axis=0)
        else:
            point_camera=np.concatenate((self.cartrace[self.i],self.cartrace[self.i]+self.velocity_list*2), axis=0)
        min_point_camera=np.min(point_camera,axis=0)
        max_point_camera=np.max(point_camera,axis=0)
        box_size=np.maximum(20,max_point_camera-min_point_camera)
        target_center=(max_point_camera+min_point_camera)/2
        target_zoom=np.minimum(self.window_width/box_size[0],self.window_height/box_size[1])*self.margin
        target_zoom=np.clip(target_zoom,self.min_zoom,self.max_zoom)
        self.scale_factor = self.scale_factor+self.smoothing_factor_zoom*(target_zoom-self.scale_factor)
        self.camera_center= self.camera_center+self.smoothing_factor_center*(target_center-self.camera_center)
        if abs(self.scale_factor - self.last_sprite_zoom) > 0.02 * abs(self.last_sprite_zoom):
            sprite_scale=(self.wheelbase*self.scale_factor)/121
            self.car_sprite=pygame.transform.smoothscale(self.scaled_sprite, (int(self.sw*sprite_scale*self.upscale_car), int(self.sh*sprite_scale*self.upscale_car)))
            self.last_sprite_zoom=self.scale_factor

    def draw_track(self):
        current_border_right=self.world_to_screen(self.borderright,self.camera_center)
        current_border_left=self.world_to_screen(self.borderleft,self.camera_center)
        
        #draw border
        pygame.draw.polygon(self.screen,color=(55, 55, 55),points=current_border_left)
        pygame.draw.polygon(self.screen,color=(0, 100, 0),points=current_border_right)
        pygame.draw.polygon(self.screen,color="White",points=current_border_left, width=3)
        pygame.draw.polygon(self.screen,color="White",points=current_border_right,width=3)

    def draw_rollouts(self):
        cartrace_prepend=self.cartrace[self.k*self.substeps]
        cartrace_prepend=cartrace_prepend[None, None,...]
        cartrace_prepend=np.repeat(cartrace_prepend,len(self.path_trajectory_array[0,...,0]),axis=0)
        Mppi_lines=np.concatenate((cartrace_prepend,self.path_trajectory_array[self.k]),axis=1)
        Mppi_lines=self.world_to_screen(Mppi_lines,self.camera_center)
        transp.fill((0, 0, 0, 0))
        for p in range(len(self.path_trajectory_array[0,...,0])-1,-1,-1):
            alpha=max(int(self.weights_array[self.k][p] * 255),20)
            if self.off_track_array[self.k][p]==False: line_color=(0, 255, 255, alpha)
            else: line_color=(255, 50, 50, 150)
            pygame.draw.lines(transp, line_color, False, Mppi_lines[p], 2)
        self.screen.blit(transp, (0, 0))

    def draw_planned_path(self):
        cartrace_prepend=self.cartrace[self.k*self.substeps]
        cartrace_prepend=cartrace_prepend[None,...]
        Optimal_path=np.concatenate((cartrace_prepend,self.current_trajectory[self.k]),axis=0)
        Optimal_path=self.world_to_screen(Optimal_path,self.camera_center)
        pygame.draw.lines(self.screen, (255, 215, 0), False, Optimal_path, 3)

    def draw_car(self):
        current_angle=np.degrees(self.heading_angle[i])
        rotated_sprite=pygame.transform.rotate(self.car_sprite, current_angle)
        current_position=self.world_to_screen(self.cartrace[i],self.camera_center)
        self.screen.blit(rotated_sprite, rotated_sprite.get_rect(center=current_position))

        
    def main_loop(self):
        sw,sh,scaled_sprite=self.setup()
        transp=pygame.Surface((self.window_width, self.window_height), pygame.SRCALPHA)
        while self.running:
            self.handle_events()
            self.screen.fill((0, 100, 0))
            self.k = min(len(self.path_trajectory_array[...,0,0])-1,self.i // self.substeps)
            #camera
            self.update_camera()
            self.draw_track()
            if self.has_rollouts:
                self.draw_rollouts()
                self.draw_planned_path()
            self.draw_car
            self.i=(self.i+2)%len(self.cartrace)
            pygame.display.flip()
            self.clock.tick(50)
        pygame.quit()