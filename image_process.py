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


class Image_process:

    # Class parameter

    # Instance method
    def __init__(self, input_image, slicescope_instance):
        self.slicescope_instance = slicescope_instance
        self.frame = input_image
        self.contours = []
        self.contour_hierarchy = []
        self.contour_coord_list = np.empty(shape = [0, 2])
        self.contour_coord_avg = np.empty(shape = [0, 2])
        self.hough_line_list = np.empty(shape = [0, 3])

    def description(self):
        return "Image Processing Functions"

    def contour(self):

        # Find Canny edges
        edged = cv.Canny(self.frame, 30, 200)
  
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  
        print("______")
  
        print("Number of Contours found = " + str(len(contours)))
  
        self.contours = contours
        self.contour_hierarchy = hierarchy

    def contours_2_coord_list(self):
        # Squash contour to a list of coordinates
        for contour in self.contours:
            for point in contour:
                self.contour_coord_list = np.append(self.contour_coord_list, point, axis = 0)

        self.contour_coord_avg = np.average(self.contour_coord_list, axis = 0)

    def hough_lines(self, rho = 0.1, theta = np.pi / 180, min_votes = 20):
        self.hough_line_list = cv.HoughLines(self.frame, rho, theta, min_votes)
        
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

