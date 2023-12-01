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

import pyvista as pv
import matplotlib.pyplot as plt

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

    #Quickly show all of the contours at each focus from saved data file.
    #data_pts = np.loadtxt('probe_point_cloud.txt', delimiter=',', dtype='int')
    #data = pv.PolyData(data_pts).delaunay_2d().elevation()
    #print(data)
    #pv.plot(data_pts)

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

    #img_proc = Image_process(slicescope)
    img_proc = Image_process()

    #Setup figure properties for matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.show(block=False)

    while True:
        # Image window is constantly updated with cam.get_frame()
        img_wnd.frame = cam.get_frame()

        time.sleep(0.1)

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img_wnd.click_coord == img_wnd.previous_click_coord:
            #Do not need to show click coordinate and overlay. Commented out.
            #img_wnd.clear_overlay()
            #img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), img_wnd.click_coord, 10, 0, 3))
            #print(f'Click Coordinate = {img_wnd.click_coord}')
            #img_wnd.click_coord_update()

            img_proc.load_frame(img_wnd.frame)   #Add the window frame to the most recent img_list (img_list[-1]). Pass img_wnd.frame into image_process class for analysis
            check_focus,check_focus_contour = img_proc.contour()  #First contour (avg,points)
            print(f"Is the probe in focus? = {check_focus}")

            if not np.isnan(check_focus):

                #Autofocus
                img_proc.load_frame(img_wnd.frame)   #Add the window frame to the most recent img_list (img_list[-1]). Pass img_wnd.frame into image_process class for analysis
                initial_avg, initial_avg_contour = img_proc.contour()  #Get contour (avg,points). 
                #print(f"Initial average = {initial_avg}")

                probe_point_cloud = []
                if not np.isnan(initial_avg):

                    #Move slicescope down until probe is out of focus
                    cnt = 0
                    while True:
                        cnt = cnt + 1
                        slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,-10_00)
                        img_wnd.frame = cam.get_frame()  # Image window is constantly updated with cam.get_frame()
                        img_proc.load_frame(img_wnd.frame) 
                        avg_down,probe_contour = img_proc.contour()  #Get contour (avg,points) 
                        print(f"avg_down = {avg_down}")

                        if np.isnan(avg_down):
                            break
                        else:
                            for d in range(len(probe_contour)):
                                probe_contour_x = probe_contour[d][0]
                                probe_contour_y = probe_contour[d][1]
                                #probe_z = slicescope.z  #This is the scan height

                                probe_point_cloud = np.append(probe_point_cloud,[probe_contour_x,probe_contour_y,slicescope.z])

                        print(f"iteration = {cnt}")

                    probe_point_cloud = np.reshape(probe_point_cloud,(-1,3))
                    probe_point_cloud.astype(int)
                    #print(probe_point_cloud)

                    #Save point cloud data to file in C:\Users\Lab directory
                    np.savetxt('probe_point_cloud.txt', probe_point_cloud, delimiter = ',')

                    #Plot probe using point cloud
                    #pv.plot(probe_point_cloud)
                    #img_proc.vispy_3D(probe_point_cloud)


                    #Move slicescope up until probe is in focus
                    #Calculate rolling average to find the minimum number of corners. This is when the probe is in focus.
                    xs = []
                    ys = []
                    moving_averages = []
                    focus_height = []
                    focus_corners = []
                    window_size = 5

                    cnt = 0
                    while True:
                        cnt = cnt + 1
                        slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,2_00)
                        img_wnd.frame = cam.get_frame()  # Image window is constantly updated with cam.get_frame()
                        img_proc.load_frame(img_wnd.frame) 
                        avg_up, avg_up_contour = img_proc.contour()
                        print(f"avg_up = {avg_up}")

                        num_corners, corners = img_proc.harris_corner(img_wnd.frame)
                        print(f"Number of corners detected = {num_corners}")

                        xs.append(cnt)
                        ys.append(num_corners)
                        focus_corners.append(corners)
                        focus_height.append(slicescope.z)

                        if len(ys) < window_size:
                            moving_averages.append(np.nan)
                        else:
                            #Calculate the average of current window
                            window_average = round(np.sum(ys[-window_size:])/window_size)
                            moving_averages.append(window_average)

                        #Plot using matplotlib
                        #These codes work to constantly update the plot in real time, but is slow.
                        #Can only have one plot open and updating in real time. The other plot will pause until the user clicks on it as the current figure.
                        ax.clear()

                        #Harris corners real time graph
                        #ax.set(title = 'Convergence on the probe tip', xlabel = 'Iteration', ylabel = 'Number of corners')
                        #ax.plot(xs[-window_size:],ys[-window_size:])
                        #ax.plot(xs,ys)

                        #Moving averages real time graph
                        ax.set(title = 'Convergence on the probe tip', xlabel = 'Window number', ylabel = 'Moving average (number of corners)')
                        #ax.plot(xs[-window_size:],moving_averages[-window_size:])
                        ax.plot(xs,moving_averages)
                        plt.pause(0.1)

                        if cnt == 100:

                            min_convergence_index = moving_averages.index(np.nanmin(moving_averages))
                            tip_in_focus_height = focus_height[min_convergence_index]
                            tip_in_focus_corners = focus_corners[min_convergence_index]
                            slicescope.moveAbsolute(slicescope.x,slicescope.y,int(tip_in_focus_height))

                            print(tip_in_focus_height)
                            print(tip_in_focus_corners)

                            #k_centroid_list,k_sse_list = img_proc.k_means(corners) as reference
                            k_centroid_list,k_sse_list = img_proc.k_means(tip_in_focus_corners)
                            k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                            #k_centroid_average = np.mean(k_centroid_list,axis=0)  #remember, coordinates need to be integer to draw on screen
                            print("k centroid list")
                            print(k_centroid_list)

                            break
                        print(f"iteration = {cnt}")

                    #Move slicescope and probe up together to remove background interference. Slicescope first, then micromanipulator.
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

                        offset_x_pixels, offset_y_pixels = img_proc.quadrant(img_tip, x2, y2)
                        dot = cv.circle(img_tip,(int(probe_tip[0]),int(probe_tip[1])),radius=10,color=(255,255,255),thickness=10)
                        cv.imshow('Probe tip', img_tip)
                        cv.waitKey(1)

                        #img_wnd.clear_overlay()
                        break

                else:
                    print('Slicescope - OUT OF FOCUS')

            else:
                print('Probe is OUT OF FOCUS')

            #img_wnd.clear_overlay()

        # Exit if img_wnd thread killed
        if not img_wnd.thread.is_alive():
            break

    #Move slicescope and probe together back to origin. Micromanipulator first, then slicescope.
    patchstar.moveRelative(patchstar.x,patchstar.y,patchstar.z,0,0,-common_z_step)
    slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,-common_z_step)

    #Use estimated tip coordinate and conversion factor to center the slicescope over the probe tip
    conversion_factor = 3_00 #um/pixel. 3.65 = 3_65 
    offset_x_micron = conversion_factor*offset_x_pixels
    offset_y_micron = conversion_factor*offset_y_pixels
    slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,int(offset_x_micron),int(offset_y_micron),0)

    img_wnd.frame = cam.get_frame()

    print("-------Check if probe is still in focus-------")
    print("")
    os.system('pause')

    #Move micromanipulator probe out. Approach = 10000_00
    a_step = 10000_00
    patchstar.approachRelative(patchstar.a,a_step)

    print("-------Switch to High Magnification-------")
    print("")
    os.system('pause')

    while True:

        img_wnd.frame = cam.get_frame()

        time.sleep(0.1)

        step = 1000_00
        
        num_step = 8

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img_wnd.click_coord == img_wnd.previous_click_coord:
            #img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), img_wnd.click_coord, 10, 0, 3))
            #print(f'Click Coordinate = {img_wnd.click_coord}')
            img_wnd.click_coord_update()
            #img_wnd.clear_overlay()

            #Move micromanipulator probe away in the Y direction to make room for High Magnification objective.
            patchstar.moveRelative(patchstar.x,patchstar.y,patchstar.z,0,8000_00,0)

            for cnt in range(num_step):

                slicescope.moveRelative(slicescope.x,slicescope.y,slicescope.z,0,0,-step)

                print(f"iteration = {cnt}")

            move_down = num_step*step
            patchstar.moveRelative(patchstar.x,patchstar.y,patchstar.z,0,0,-move_down)

            #Move micromanipulator probe back in the Y direction to position probe to approach water and be seen under High Magnification.
            patchstar.moveRelative(patchstar.x,patchstar.y,patchstar.z,0,-8000_00,0)

            img_wnd.frame = cam.get_frame()

            #Move micromanipulator probe in. Approach = -10000_00
            patchstar.approachRelative(patchstar.a,-a_step)

            if cnt == (num_step-1):
                break

    img_wnd.frame = cam.get_frame()

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