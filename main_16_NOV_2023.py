#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 13:57:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

import sys
import os
import string
import numpy as np
import cv2 as cv
import threading
import time

#Classes
from slicescope import Slicescope
from patchstar import Patchstar
from pvcam import PVCAM
#from condenser import Condenser
from image_window import Image_window
from image_process import Image_process


def main():

    global slicescope, patchstar

    print('-----MAIN-----')

    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')

    slicescope.coordinates()
    print(f"Slicescope values X = {slicescope.x}, Y = {slicescope.y}, Z = {slicescope.z}")

    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM7')

    patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar.x}, Y = {patchstar.y}, Z = {patchstar.z}, A = {patchstar.a}")

    #Run calibration for max and min values for coordinate systems
    slicescope.calibration()
    patchstar.calibration()

    print('-----Camera-----')

    cam = PVCAM()

    #This is the image Window, not the Camera, nor the image frame
    global img_wnd
    img_wnd = Image_window(size = cam.size)

    img_wnd.set_live_thread()
    img_wnd.thread.start()

    # Wait for image window ready
    img_wnd.wait_window_ready()
    img_wnd.set_mouse_response()

    img_proc = Image_process(slicescope)

    while True:
        # Generate random pixel map at a rate
        img_wnd.frame = cam.get_frame()

        time.sleep(0.1)

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img_wnd.click_coord == img_wnd.previous_click_coord:
            #img_wnd.clear_overlay()
            img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), img_wnd.click_coord, 10, 0, 3))
            print(f'Click Coordinate = {img_wnd.click_coord}')
            img_wnd.click_coord_update()

            img_proc.load_frame(img_wnd.frame)   #Add the window frame to the most recent img_list (img_list[-1])
            check_focus = img_proc.contour()  #First contour
            print(f"Is the probe in focus? = {check_focus}")

            if not np.isnan(check_focus):

                #Autofocus
                img_proc.load_frame(img_wnd.frame)   #Add the window frame to the most recent img_list (img_list[-1])
                initial_avg = img_proc.contour()  #First contour
                #print(f"Initial average = {initial_avg}")

                if not np.isnan(initial_avg):

                    #Move slicescope down until probe is out of focus
                    cnt = 0
                    while True:
                        cnt = cnt + 1
                        slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,-10_00)
                        img_wnd.frame = cam.get_frame()
                        img_proc.load_frame(img_wnd.frame) 
                        avg_down = img_proc.contour()
                        print(f"avg_down = {avg_down}")
                        
                        if np.isnan(avg_down):
                            break
                        print(f"iteration = {cnt}")

                    #Move slicescope up until probe is in focus
                    cnt = 0
                    while True:
                        cnt = cnt + 1
                        slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,2_00)
                        img_wnd.frame = cam.get_frame()
                        img_proc.load_frame(img_wnd.frame) 
                        avg_up = img_proc.contour()
                        print(f"avg_up = {avg_up}")

                        num_corners, corners = img_proc.harris_corner(img_wnd.frame)
                        print(f"Number of corners detected = {num_corners}")

                        if num_corners <= 3:
                            
                            k_centroid_list,k_sse_list = img_proc.k_means(corners)
                            k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                            k_centroid_average = np.mean(k_centroid_list,axis=0)  #remember, coordinates need to be integer to draw on screen
                            print("k centroid list")
                            print(k_centroid_list)

                            break
                        print(f"iteration = {cnt}")

                else:
                    print('Slicescope - OUT OF FOCUS')

                #Move slicescope and probe up together to remove background interference
                common_z_step = min(slicescope.z_delta,patchstar.z_delta)
                slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,common_z_step)
                patchstar.moveRelative(patchstar.x,patchstar.y,patchstar.z,0,0,common_z_step)

                #Find probe direction and tip
                img_tip = cam.get_frame()
                tip_boundary,tip_points = img_proc.detect_probe(img_tip)

                (x1,y1),(x2,y2) = img_proc.detect_probe_direction(img_tip, tip_boundary, tip_points)

                #(x2,y2) is your estimated tip coordinate
                if not (np.isnan(x2) or np.isnan(y2)):

                    centroid_distance = []

                    # Loop over each data point
                    for i in range( len(k_centroid_list) ):
                        x = k_centroid_list[i]

                        # Calculate distance from each k centroid to estimated probe tip (arrow tip)
                        dist = img_proc.compute_L2_distance(x, [x2,y2])
                        centroid_distance.append(dist)

                    #print(centroid_distance)

                    closest_centroid_index =  centroid_distance.index(min(centroid_distance))
                    probe_tip = k_centroid_list[closest_centroid_index]
                    #print("probe tip calculation using min L2 distance")
                    #print(probe_tip)

                    img_proc.quadrant(img_tip, x2, y2)
                    dot = cv.circle(img_tip,(int(probe_tip[0]),int(probe_tip[1])),radius=10,color=(255,255,255),thickness=10)
                    cv.imshow('Probe tip', img_tip)
                    cv.waitKey(1)

            else:
                print('Probe is OUT OF FOCUS')

            img_wnd.clear_overlay()

        # Exit if img_wnd thread killed
        if not img_wnd.thread.is_alive():
            break
    
    #Close and disconnect all equipment

    cam.disconnect()
    
    patchstar.close()

    slicescope.close()

    print('-----End of MAIN-----')
    
    """
    #Run Condenser commands

    print('-----Condenser-----')
    condenser = Condenser('COM4')

    #condenser_z = condenser.coordinates()
    #print(f"Current Condenser value Z = {condenser_z}")

    #z coordinates only. - is UP. + is DOWN. Only Condenser is flipped.
    # move absolute (abs_z)
    condenser_z = condenser.moveAbsolute(1560000)    #Move Condenser down to lowest position.
    print(f"Current Condenser value Z = {condenser_z}")

    # move relative (current_z,rel_z)
    #condenser_z = condenser.moveRelative(condenser_z,-10000) #Negative numbers = UP.

    condenser.close()
    """

if __name__ == '__main__':
    main()