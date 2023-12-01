#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri December 1 13:00:00 2023

@author: Dr. Vincent Lee
Bioptix Ltd.

"""

from pynput import keyboard

class Camera:

    # Class parameter

    # Instance method
    def __init__(self, slicescope_instance):
        self.slicescope_instance = slicescope_instance

    def description(self):
        return "Camera control functions and display"

    def key_press(self, key):

            step = 1000_00

            if key == keyboard.Key.up:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,-step,0)

            if key == keyboard.Key.down:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,step,0)

            if key == keyboard.Key.left:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,-step,0,0)

            if key == keyboard.Key.right:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,step,0,0)

            if key == keyboard.Key.page_up:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,step)

            if key == keyboard.Key.page_down:
                self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-step)

    def box(self, camera_frame):

        height,width = camera_frame.shape

        # [0] = 1024, [1] = 1376
        #print(height) #[0]
        #print(width)  #[1]
        #center_of_screen = [int(width/2),int(height/2)]

        center_of_screen_x = int(width/2)
        center_of_screen_y = int(height/2)

        #High Magnification is 10x the Low Magnification. Region Of Interest (ROI) = 10% of total frame pixels
        box_width = 0.1*width
        box_height = 0.1*height

        top_left_corner_x = center_of_screen_x - int(0.5*box_width)
        top_left_corner_y = center_of_screen_y - int(0.5*box_height)

        top_right_corner_x = center_of_screen_x + int(0.5*box_width)
        top_right_corner_y = center_of_screen_y - int(0.5*box_height)

        bottom_left_corner_x = center_of_screen_x - int(0.5*box_width)
        bottom_left_corner_y = center_of_screen_y + int(0.5*box_height)

        bottom_right_corner_x = center_of_screen_x + int(0.5*box_width)
        bottom_right_corner_y = center_of_screen_y + int(0.5*box_height)

        #Show ROI on screen
        self.cross(camera_frame,center_of_screen_x,center_of_screen_y)
        self.cross(camera_frame,top_left_corner_x,top_left_corner_y)
        self.cross(camera_frame,top_right_corner_x,top_right_corner_y)
        self.cross(camera_frame,bottom_left_corner_x,bottom_left_corner_y)
        self.cross(camera_frame,bottom_right_corner_x,bottom_right_corner_y)

        return camera_frame

    def cross(self, window_frame, x_coord, y_coord):

        # height,width are window_frame dimensions
        # [0] = 1024, [1] = 1376
        #center_of_screen = [int(width/2),int(height/2)] = [x,y]

        #Vertical line

        window_frame[y_coord-2][x_coord] = 0
        window_frame[y_coord-2][x_coord] = 0
        window_frame[y_coord-2][x_coord] = 0
        window_frame[y_coord-2][x_coord] = 0
        window_frame[y_coord-2][x_coord] = 0

        window_frame[y_coord-1][x_coord] = 0
        window_frame[y_coord-1][x_coord] = 0
        window_frame[y_coord-1][x_coord] = 0
        window_frame[y_coord-1][x_coord] = 0
        window_frame[y_coord-1][x_coord] = 0

        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0

        window_frame[y_coord+1][x_coord] = 0
        window_frame[y_coord+1][x_coord] = 0
        window_frame[y_coord+1][x_coord] = 0
        window_frame[y_coord+1][x_coord] = 0
        window_frame[y_coord+1][x_coord] = 0

        window_frame[y_coord+2][x_coord] = 0
        window_frame[y_coord+2][x_coord] = 0
        window_frame[y_coord+2][x_coord] = 0
        window_frame[y_coord+2][x_coord] = 0
        window_frame[y_coord+2][x_coord] = 0

        #Horizontal line

        window_frame[y_coord][x_coord-2] = 0
        window_frame[y_coord][x_coord-2] = 0
        window_frame[y_coord][x_coord-2] = 0
        window_frame[y_coord][x_coord-2] = 0
        window_frame[y_coord][x_coord-2] = 0

        window_frame[y_coord][x_coord-1] = 0
        window_frame[y_coord][x_coord-1] = 0
        window_frame[y_coord][x_coord-1] = 0
        window_frame[y_coord][x_coord-1] = 0
        window_frame[y_coord][x_coord-1] = 0

        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0
        window_frame[y_coord][x_coord] = 0

        window_frame[y_coord][x_coord+1] = 0
        window_frame[y_coord][x_coord+1] = 0
        window_frame[y_coord][x_coord+1] = 0
        window_frame[y_coord][x_coord+1] = 0
        window_frame[y_coord][x_coord+1] = 0

        window_frame[y_coord][x_coord+2] = 0
        window_frame[y_coord][x_coord+2] = 0
        window_frame[y_coord][x_coord+2] = 0
        window_frame[y_coord][x_coord+2] = 0
        window_frame[y_coord][x_coord+2] = 0
