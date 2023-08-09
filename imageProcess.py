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


class imageProcess:

    # Class parameter

    # Instance method
    def __init__(self,pvcam_instance,slicescope_instance):
        self.pvcam_instance = pvcam_instance
        self.slicescope_instance = slicescope_instance

    def description(self):
        return "Image Processing Functions"

    def contour(self):

        input_image = self.pvcam_instance.get_frame()

        # Find Canny edges
        edged = cv.Canny(input_image, 30, 200)
  
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  
        point_list=np.empty(shape=[0,2])

        for contour in contours:
            for point in contour:
                point_list=np.append(point_list,point,axis=0)

        avg = np.average(point_list, axis=0)

        if not np.isnan(np.average(point_list)):
            minx_idx = np.argmin(point_list[:, 0])
            minx_idx_coord = point_list[minx_idx]
            tip_coord = np.array([[int(minx_idx_coord[1]), int(minx_idx_coord[0])]])

        else:
            tip_coord = avg

        print("______")
  
        print("Number of Contours found = " + str(len(contours)))
  
        return tip_coord

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

