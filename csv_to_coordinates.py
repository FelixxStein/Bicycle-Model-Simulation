from pathlib import Path
import pandas as pd
import numpy as np

class csv_to_coordinates:
    def __init__(self,csv_racetrack):
        REPO_ROOT = Path(__file__).resolve().parent
        csv_path = REPO_ROOT /"data" /"racetracks" /csv_racetrack
        self.df = pd.read_csv(csv_path)

    def calculate_border_points(self, degree, array):
        s=np.sin(np.radians(degree))
        c=np.cos(np.radians(degree))   
        RR = np.array([[c, -s], [s, c]]) 
        new_array=[]
        points=self.get_midline()
        for i in range(len(points)-1):
            diff=points[i+1]-points[i]
            normalvector=diff/np.linalg.norm(diff)
            new_array.append(points[i]+array[i]*(RR@normalvector))
        diff=points[0]-points[len(points)-1]
        normalvector=diff/np.linalg.norm(diff)
        new_array.append(points[0]+array[len(points)-1]*(RR@normalvector))
        new_array=np.array(new_array)
        return new_array

    def get_midline(self):
        x=self.df.iloc[:, 0].tolist()
        y=self.df.iloc[:, 1].tolist()
        x=np.array(x)
        y=np.array(y)
        points = np.column_stack((x, y))
        return points

    def get_border_left(self):
        w_tr_left_m=self.df.iloc[:, 3].tolist()
        return self.calculate_border_points(-90, w_tr_left_m)


    def get_border_right(self):
        w_tr_right_m=self.df.iloc[:, 2].tolist()
        return self.calculate_border_points(90,w_tr_right_m)