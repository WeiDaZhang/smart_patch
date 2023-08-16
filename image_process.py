#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:23:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""
from itertools import combinations
import cv2 as cv
import numpy as np

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

    def hough_lines(self, rho = 1, theta = np.pi / 180 / 10, min_votes = 1):
        #self.img_list[-1].hough_line_list = cv.HoughLinesP(self.img_list[-1].edge, rho, theta, min_votes, None, 150, 50)
        hough_line_list = cv.HoughLines(self.img_list[-1].edge, rho, theta, min_votes, None, 100, 100)
        # HoughLines returns n x 1 x 2 matrix, reduce (reshape) to n x 2
        self.img_list[-1].houghline_rhotheta_list = np.reshape(hough_line_list, (len(hough_line_list),2))

    def scale_points_2_frame(self, point_list, size = (1024, 1024)):
        scale = [1, 1]
        offset = [np.min(point_list[:, 0]), np.min(point_list[:, 1])]
        point_list[:, 0] = point_list[:, 0] - np.min(point_list[:, 0])
        point_list[:, 1] = point_list[:, 1] - np.min(point_list[:, 1])
        scale[0] = (size[0] - 1) / np.max(point_list[:, 0])
        scale[1] = (size[1] - 1) / np.max(point_list[:, 1])
        point_list[:, 0] = point_list[:, 0] * scale[0]
        point_list[:, 1] = point_list[:, 1] * scale[1]
        return point_list, scale, offset

    def point_list_2_frame(self, point_list, size = (1024, 1024)):
        frame = np.zeros(shape = size)
        for idx in range(0, len(point_list)):
            frame[np.floor(point_list[idx, 0]).astype(int), np.floor(point_list[idx, 1]).astype(int)] += 1
        return frame

    def frame_2_point_list(self, frame, contrast = 0.5):
        frame = frame - np.min(frame)
        frame = frame / np.max(frame)
        point_list = np.empty(shape = (0, 2))
        for idx_row in range(0, frame.shape[0]):
            for idx_col in range(0, frame.shape[1]):
                if frame[idx_row, idx_col] > contrast:
                    point_list = np.append(point_list, [[idx_row, idx_col]], axis = 0)
        return point_list

    def cartToPolar(self, coord_list, angleInDegress = False):
        mag = np.sqrt(np.square(coord_list[:, 0]) + np.square(coord_list[:, 1]))
        # matrix-row = y
        # matrix-col = x
        ang = np.arctan2(coord_list[:, 0], coord_list[:, 1])
        if angleInDegress:
            ang = ang / np.pi * 180 
        return np.column_stack((mag.T, ang.T))

    def hough_2_point_lines(self, rho_theta_list):
        sin_minus_theta_list = np.sin(-rho_theta_list[:, 1])
        cos_minus_theta_list = np.cos(-rho_theta_list[:, 1])
        rho_sin_product_list = rho_theta_list[:, 0] * sin_minus_theta_list
        rho_cos_product_list = rho_theta_list[:, 0] * cos_minus_theta_list
        combination_idx_list = np.stack(np.triu_indices(len(rho_theta_list), k=1), axis=-1)
        actan_nominator = np.diff(rho_sin_product_list[combination_idx_list], axis = 1)
        actan_denominator = np.diff(rho_cos_product_list[combination_idx_list], axis = 1)
        actan_nonzero_idx_list = np.flatnonzero(actan_denominator)
        cross_theta = np.pi/2 - np.arctan(actan_nominator[actan_nonzero_idx_list] / actan_denominator[actan_nonzero_idx_list])
        rho_theta_combination_list = rho_theta_list[combination_idx_list[actan_nonzero_idx_list, 0], :]
        cross_rho = rho_theta_combination_list[:, 0] * np.cos(cross_theta[:, 0] - rho_theta_combination_list[:, 1])
        cross_rho.shape = (cross_rho.size, 1)
        self.img_list[-1].houghline_rhotheta_list = np.column_stack((cross_rho, cross_theta))
        
        #rho_theta_2_point_list = combinations(rho_theta_list, 2)
        #cross_rho_theta_list = np.empty(shape = (0, 2))
        #for rho_theta_2_point in list(rho_theta_2_point_list):
        #    #input(rho_theta_2_point)
        #    actan_nominator = rho_theta_2_point[0][0]*np.sin(-rho_theta_2_point[0][1]) - rho_theta_2_point[1][0]*np.sin(-rho_theta_2_point[1][1])
        #    actan_denominator = rho_theta_2_point[0][0]*np.cos(-rho_theta_2_point[0][1]) - rho_theta_2_point[1][0]*np.cos(-rho_theta_2_point[1][1])
        #    if actan_denominator == 0:
        #        continue
        #    cross_theta = np.pi/2 - np.arctan(actan_nominator / actan_denominator)
        #    cross_rho = rho_theta_2_point[0][0]*np.cos(cross_theta - rho_theta_2_point[0][1])
        #    cross_rho_theta_list = np.append(cross_rho_theta_list, [[cross_rho, cross_theta]], axis = 0)
        #    #print([cross_rho, cross_theta])
        #    #if len(cross_rho_theta_list) > 10:
        #    #    break
        #self.img_list[-1].houghline_rhotheta_list = cross_rho_theta_list
        #return cross_rho_theta_list

    def hist_2d_coord(self, coord_list, size = (1024, 1024), resolution = 10):
        coord_list = np.array(coord_list)
        print(f"Coordinate List: \n{coord_list}")
        
        hist_bins = np.array([np.ceil(size[0]/resolution), np.ceil(size[1]/resolution)]).astype(int)
        # Histogram2d input: x first, y second, output: row = x, col = y
        coord_hist_T, coord_edge_x, coord_edge_y = np.histogram2d(coord_list[:, 1],   # second column in coorinate list is x
                                                                coord_list[:, 0],   # first column in coorinate list is y
                                                                hist_bins)
        #print(f"hist: {coord_hist}")
        #print(f"edge x: {coord_edge_x}")
        #print(f"edge y: {coord_edge_y}")
            
        return coord_hist_T.T

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

