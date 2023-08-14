#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:23:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

import cv2 as cv
import numpy as np

from pyvcam import pvc
from pyvcam.camera import Camera
from pyvcam import constants

class Image_object:

    # Class parameter

    # Instance method
    def __init__(self, input_image):
        self.frame = input_image
        self.edge = []
        self.contours = []
        self.contour_hierarchy = []
        self.contour_coord_list = np.empty(shape = [0, 2])
        self.contour_coord_avg = []
        self.houghline_p_list = []
        self.houghline_rhotheta_list = []

    def description(self):
        return "Image Object"


class Image_process:

    # Class parameter

    # Instance method
    def __init__(self, slicescope_instance):
        self.slicescope_instance = slicescope_instance
        self.img_list = []

    def description(self):
        return "Image Processing Functions"

    def load_frame(self, frame):
        self.img_list.append(Image_object(frame))

    def contour(self):

        # Find Canny edges
        self.img_list[-1].edge = cv.Canny(self.img_list[-1].frame, 30, 200)
  
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(self.img_list[-1].edge, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  
        print("______")
  
        print("Number of Contours found = " + str(len(contours)))
  
        self.img_list[-1].contours = contours
        self.img_list[-1].contour_hierarchy = hierarchy

    def contours_2_coord_list(self):
        # Squash contour to a list of coordinates
        for contour in self.img_list[-1].contours:
            for point in contour:
                self.img_list[-1].contour_coord_list = np.append(self.img_list[-1].contour_coord_list, point, axis = 0)

        self.img_list[-1].contour_coord_avg = np.average(self.img_list[-1].contour_coord_list, axis = 0)

    def hough_lines(self, rho = 0.1, theta = np.pi / 180 / 100, min_votes = 10):
        #self.img_list[-1].hough_line_list = cv.HoughLinesP(self.img_list[-1].edge, rho, theta, min_votes, None, 150, 50)
        hough_line_list = cv.HoughLines(self.img_list[-1].edge, rho, theta, min_votes)
        self.img_list[-1].houghline_rhotheta_list = np.reshape(hough_line_list, (len(hough_line_list),2))

    def scale_points_2_frame(self, point_list, size = (1024, 1024)):
        scale = [1, 1]
        offset = [np.min(point_list[:, 0], np.min(point_list[:, 1]]
        point_list[:, 0] = point_list[:, 0] - np.min(point_list[:, 0])
        point_list[:, 1] = point_list[:, 1] - np.min(point_list[:, 1])
        scale[0] = (size[0] - 1) / np.max(point_list[:, 0])
        scale[1] = (size[1] - 1) / np.max(point_list[:, 1])
        point_list[:, 0] = point_list[:, 0] * scale[0]
        point_list[:, 1] = point_list[:, 1] * scale[1]
        return point_list, scale, offset

    def point_list_2_frame(self, point_list, size = (1024, 1024), scale_points = True):
        frame = np.zeros(shape = size)
        if scale_points:
            point_list = scale_points_2_frame(point_list, size)[0]
        for idx in range(0, len(point_list)):
            frame[np.floor(point_list[idx, 0]).astype(int), np.floor(point_list[idx, 1]).astype(int)] += 1
        return frame

    def autofocus(self,slicescope_x,slicescope_y,slicescope_z):

        #Move slicescope down until probe is out of focus
        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = self.slicescope_instance.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,-5_00)
            tip_coord = self.contour()
            print(tip_coord)
            if np.isnan(np.average(tip_coord)):
                break

        #Move slicescope up until probe is in focus
        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = self.slicescope_instance.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = self.contour()
            print(tip_coord)
            if not np.isnan(np.average(tip_coord)):
                break
            print(f"iteration = {cnt}")

        #Find the tip of the probe
        tip_coord_list = np.empty(shape=[0,2])
        for idx in range(0,20,1):
            slicescope_x,slicescope_y,slicescope_z = self.slicescope_instance.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = self.contour()
            print(f"tip = {tip_coord}")
            if not np.isnan(np.average(tip_coord)):
                tip_coord_list = np.append(tip_coord_list,tip_coord,axis=0)
        
        print("______")
        
        print(tip_coord_list) # (y values ,x values)        

        return tip_coord_list

