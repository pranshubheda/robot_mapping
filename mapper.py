#!/usr/bin/python
'''
  Some Tkinter/PIL code to pop up a window with a gray-scale
  pixel-editable image, for mapping purposes.  Does not run
  until you fill in a few things.

  Does not do any mapping.

  Z. Butler, 3/2016, updated 3/2018
'''

import Tkinter as tk
from PIL import Image, ImageTk
import random
import rospy
from sensor_msgs.msg import LaserScan
import csv
import numpy as np
import math
import sys

# a reasonable size? depends on the scale of the map and the
# size of the environment, of course:
MAPSIZE = 400

class Mapper(tk.Frame):    

    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("I'm the map!")
        self.master.minsize(width=MAPSIZE,height=MAPSIZE)

        # makes a grey-scale image filled with 50% grey pixels
        self.themap = Image.new("L",(MAPSIZE,MAPSIZE),128)
        self.mapimage = ImageTk.PhotoImage(self.themap)

        # this gives us directly memory access to the image pixels:
        self.mappix = self.themap.load()
        # keeping the odds separately saves one step per cell update:
        self.oddsvals = [[0 for _ in range(MAPSIZE)] for _ in range(MAPSIZE)]

        self.canvas = tk.Canvas(self,width=MAPSIZE, height=MAPSIZE)

        self.map_on_canvas = self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)
        self.canvas.pack()
        self.current_pose = []  
        self.current_grid_coordinates = (MAPSIZE/2, MAPSIZE/2)
        self.sonar_data = []
        self.laser_data = []
        self.sonar_angles = [math.radians(-90), math.radians(-50), math.radians(-30), math.radians(-15), math.radians(15), math.radians(30), math.radians(50), math.radians(90)]
        self.pack()

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)       
        self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)


    def sonar_update(self):
        local_current_sonar_data = self.sonar_data
        local_current_pose = self.current_pose
        local_current_grid_coordinates = self.current_grid_coordinates
        # -50 to 50
        center_wedge_sonar_data = local_current_sonar_data[1:-2]
        angle_ctr=1
        thickness_buffer = 0.8

        x1, y1 = local_current_grid_coordinates[0], local_current_grid_coordinates[1]
        for sonar_point_distance in center_wedge_sonar_data:
            local_point_coordinates = self.find_local_coordinates(local_current_pose, sonar_point_distance, self.sonar_angles[angle_ctr])
            x2, y2 = self.world_to_grid(local_point_coordinates)
            points = self.bresenham(x1, y1, x2, y2)
            angle_proportional = angle_ctr
            if angle_proportional > 3:
                angle_proportional%=3
                angle_proportional/=3
            for j in range(len(points)-1):
                distance_proportional = (j+1)/(len(points)-1)
                point = points[j]
                x = int(point[0])
                y = int(point[1])
                proportional_odds = (0.45/0.55) * angle_proportional * distance_proportional
                self.oddsvals[x][y] = self.oddsvals[x][y] + np.log(proportional_odds)
                odds = np.exp(self.oddsvals[x][y]) 
                prob = (1.0/(1+odds))
                pixel_value = prob * 255 
                # pixel_value = (1.0/(1+self.oddsvals[x][y])) * 255 
                self.mappix[x,y] = int(pixel_value)
            
            if sonar_point_distance <= 2.5:
                boundary_start_point_pose = [local_point_coordinates[0], local_point_coordinates[1], local_current_pose[-1]]
                boundary_end_point_coordinates = self.find_local_coordinates(boundary_start_point_pose, thickness_buffer, self.sonar_angles[angle_ctr])
                x3, y3 = self.world_to_grid(boundary_end_point_coordinates)
                points = self.bresenham(x2, y2, x3, y3) 
                for j in range(len(points)):
                    point = points[j]
                    x = int(point[0])
                    y = int(point[1])
                    proportional_odds = (0.70/0.30) * angle_proportional
                    # self.oddsvals[x][y] = self.oddsvals[x][y] * (0.70/0.30) * angle_proportional
                    self.oddsvals[x][y] = self.oddsvals[x][y] + np.log(proportional_odds)
                    odds = np.exp(self.oddsvals[x][y]) 
                    prob = (1.0/(1+odds))
                    pixel_value = prob * 255 
                    # pixel_value = (1.0/(1+self.oddsvals[x][y])) * 255 
                    self.mappix[x,y] = int(pixel_value)
            angle_ctr+=1

  
        # print('Updating Image after finishing scan')

    def laser_update(self):
        local_current_laser_data = self.laser_data
        local_current_pose = self.current_pose
        local_current_grid_coordinates = self.current_grid_coordinates

        center_wedge_laser_data = local_current_laser_data
        thickness_buffer = 0.05
        
        angle_proportional = 0
        if angle_proportional > 320:
            angle_proportional%=320
            angle_proportional/=320

        x1, y1 = local_current_grid_coordinates[0], local_current_grid_coordinates[1]
        for i, laser_point_distance in enumerate(center_wedge_laser_data):
            if laser_point_distance == float('inf'):
                laser_point_distance = 5
            laser_angle = self.find_laser_edge_angle(i)
            laser_point_coordinates = self.find_local_coordinates(local_current_pose, laser_point_distance, laser_angle)
            x2, y2 = self.world_to_grid(laser_point_coordinates)
            #update free space
            points = self.bresenham(x1, y1, x2, y2)

            for j in range(len(points)):
                point = points[j]
                x = int(point[0])
                y = int(point[1])
                odds = (0.45/0.55) 
                self.oddsvals[x][y] = self.oddsvals[x][y] + np.log(odds)
                odds = np.exp(self.oddsvals[x][y])
                prob = (1.0/(1+odds))
                pixel_value = prob * 255 
                self.mappix[x,y] = int(pixel_value)
            
            if laser_point_distance <= 10:
                boundary_start_point_pose = [laser_point_coordinates[0], laser_point_coordinates[1], local_current_pose[-1]]
                boundary_end_point_coordinates = self.find_local_coordinates(boundary_start_point_pose, thickness_buffer, laser_angle)
                x3, y3 = self.world_to_grid(boundary_end_point_coordinates)
                points = self.bresenham(x2, y2, x3, y3)

                for j in range(len(points)):
                    point = points[j]
                    x = int(point[0])
                    y = int(point[1])
                    odds = (0.55/0.45)
                    self.oddsvals[x][y] = self.oddsvals[x][y] + np.log(odds)
                    odds = np.exp(self.oddsvals[x][y]) 
                    prob = (1.0/(1+odds))
                    pixel_value = prob * 255 
                    # pixel_value = (1.0/(1+self.oddsvals[x][y])) * 255 
                    self.mappix[x,y] = int(pixel_value)

    def find_laser_edge_angle(self, laser_index):
        if laser_index > 320:
            mag = laser_index%320
            edge_angle = (56.0/640.0) * mag
        else:
            mag = laser_index%320
            edge_angle = (56.0/640.0) * (320-mag) * -1
        return math.radians(edge_angle)

        
    def bresenham(self, x0, y0, x1, y1):
        """Yield integer coordinates on the line from (x0, y0) to (x1, y1).
        Input coordinates should be integers.
        The result will contain both the start and the end point.
        Plagiarism not intended https://github.com/encukou/bresenham
        """
        points = []
        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0

        for x in range(dx + 1):
            points.append((x0 + x*xx + y*yx, y0 + x*xy + y*yy))
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy
        return points

    def find_local_coordinates(self, local_current_pose, point_distance, alpha):
        theta = local_current_pose[-1]
        local_coordinates = (math.cos(theta+alpha)*point_distance, math.sin(theta+alpha)*point_distance)
        local_coordinates = (local_current_pose[0]+local_coordinates[0], local_current_pose[1]+local_coordinates[1])
        return local_coordinates

    def world_to_grid(self, point):
        grid_center_coordinates = (MAPSIZE/2, MAPSIZE/2)
        scaling_factor = MAPSIZE/2
        delta_x = (point[0]/15.0) * scaling_factor
        delta_y = (point[1]/15.0) * scaling_factor
        grid_coordinates = None
        grid_coordinates = (int(round(grid_center_coordinates[0]+delta_x)), int(round(grid_center_coordinates[1]-delta_y)))
        return grid_coordinates

def main(sensor, file_path):
    rospy.init_node("mapper")

    root = tk.Tk()
    m = Mapper(master=root,height=MAPSIZE,width=MAPSIZE)
    print('Generating map.... ')
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:

            m.current_pose = np.array(row[:3]).astype(np.float)
            m.current_grid_coordinates = m.world_to_grid((m.current_pose[0], m.current_pose[1]))
            m.sonar_data = np.array(row[3:3+8]).astype(np.float)
            m.laser_data = np.array(row[3+8:]).astype(np.float)
            
            if sensor == 'sonar':
                m.sonar_update()
            elif sensor == 'laser':
                m.laser_update()
            elif sensor == 'both':
                m.laser_update()
                m.sonar_update()

    m.update_image()
    root.mainloop()

if __name__ == "__main__":
    sensor = sys.argv[1]
    file_path = sys.argv[2]
    main(sensor, file_path)
