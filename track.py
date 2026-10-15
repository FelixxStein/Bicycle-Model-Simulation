from pathlib import Path
import pandas as pd
import numpy as np

class track:
    def __init__(self,csv_racetrack, curvature_lookahead_distance=2,lookahead_closest_index=40,lookbefore_closest_index=5):
        REPO_ROOT = Path(__file__).resolve().parent
        csv_path = REPO_ROOT /"data" /"racetracks" /csv_racetrack
        self.df = pd.read_csv(csv_path)
        self.lookahead_closest_index=lookahead_closest_index
        self.lookbefore_closest_index=lookbefore_closest_index
        #calculate midline
        x=self.df.iloc[:, 0].tolist()
        y=self.df.iloc[:, 1].tolist()
        x=np.array(x)
        y=np.array(y)
        self.midline = np.column_stack((x, y))
        #calculate border points
        w_tr_left_m=self.df.iloc[:, 3].tolist()
        w_tr_right_m=self.df.iloc[:, 2].tolist()
        self.w_tr_left_m=np.array(w_tr_left_m)
        self.w_tr_right_m=np.array(w_tr_right_m)
        w_tr_left_column=np.reshape(w_tr_left_m,(len(w_tr_left_m),1))
        w_tr_right_column=np.reshape(w_tr_right_m,(len(w_tr_right_m),1))
        normal=[]
        s=np.sin(np.radians(90))
        c=np.cos(np.radians(90)) 
        RR = np.array([[c, -s], [s, c]])
        #calculate cumulative point distance and normals in one loop
        cumulative_point_distance=[]
        angle=[]
        dist_accumulated=0
        tangent=[]
        for i in range(len(self.midline)):
            #cummulated distance
            cumulative_point_distance.append(dist_accumulated)
            segment=self.midline[(i+1)%len(self.midline)]-self.midline[i%len(self.midline)]
            tangent_segment=(segment/np.linalg.norm(segment))
            tangent.append(tangent_segment)
            normal.append(RR@tangent_segment)
            angle_segment = np.arctan2(tangent_segment[1],tangent_segment[0])
            angle.append(angle_segment)
            dist_accumulated+=np.sqrt(np.sum(segment**2))

        self.normal=np.array(normal)
        self.tangent=np.array(tangent)
        self.cumulative_point_distance=np.array(cumulative_point_distance)
        self.angle=np.array(angle)
        self.track_length=dist_accumulated
        #calculate curvature
        curvature=[]
        for i in range(len(self.midline)):
            a=(i-curvature_lookahead_distance)%len(self.midline)
            b=(i+curvature_lookahead_distance)%len(self.midline)
            turn=angle[b]-angle[a]
            if turn>np.pi:
                turn-=2*np.pi
            elif turn<-np.pi:
                turn+=2*np.pi
            distance=(self.forward_distance(a,b))%self.track_length
            curvature_segment=turn/distance
            curvature.append(curvature_segment)
        self.curvature=np.array(curvature)

        self.border_right=self.midline-w_tr_right_column*self.normal
        self.border_left=self.midline+w_tr_left_column*self.normal
        

    def get_closest_index(self, starting_index, position):
        closest_index=starting_index
        for i in range(starting_index-self.lookbefore_closest_index, starting_index+self.lookahead_closest_index):
            if np.sum((position - self.midline[i%len(self.midline)])**2)<np.sum((position - self.midline[closest_index])**2):
                closest_index=i%len(self.midline)
        return closest_index

    def forward_distance(self, a, b):
        return (self.cumulative_point_distance[b]-self.cumulative_point_distance[a])%self.track_length

    def locate(self,position, start_index,margin=1):
        index=self.get_closest_index(start_index, position)
        difference=position-self.midline[index]
        along=np.sum(difference*self.tangent[index], axis=-1)
        index=np.where(along<0, index-1, index)
        index=(index)%len(self.midline)
        difference=position-self.midline[index]
        along=np.sum(difference*self.tangent[index], axis=-1)
        lateral=np.sum(difference*self.normal[index], axis=-1)
        progress=self.cumulative_point_distance[index]+along
        ontrack=self.on_track(index,lateral,margin)
        return index, progress,lateral,ontrack

    def on_track(self,index,lateral, margin):
        if -(self.w_tr_right_m[index]-margin)<= lateral and self.w_tr_left_m[index]-margin>=lateral:
            return True
        else:
            return False

    def progress_difference(self,progress_start, progress_end):
        progress=progress_end-progress_start
        progress=np.where(progress<-self.track_length/2,progress+self.track_length, progress)
        progress=np.where(progress>self.track_length/2,progress-self.track_length, progress)
        return progress



    def get_midline(self):
        return self.midline

    def get_border_left(self):
        return self.border_left
        
    def get_border_right(self):
        return self.border_right

    def get_cumulative_point_distance(self):
        return self.cumulative_point_distance

    def get_curvature(self):
        return self.curvature

    def get_track_length(self):
        return self.track_length

