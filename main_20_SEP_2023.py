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
import random
import matplotlib.pyplot as plt

#Classes
from slicescope import Slicescope
from patchstar import Patchstar
from pvcam import PVCAM
#from condenser import Condenser
from image_window import Image_window
from image_process import Image_process


def main():

    print('-----MAIN-----')

    #Global coordinates for the system
    global slicescope_x, slicescope_y, slicescope_z, patchstar_x, patchstar_y, patchstar_z, patchstar_a #, condenser_z


    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')
    
    slicescope_x,slicescope_y,slicescope_z = slicescope.coordinates()
    print(f"Slicescope values X = {slicescope_x}, Y = {slicescope_y}, Z = {slicescope_z}")


    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM7')

    patchstar_x,patchstar_y,patchstar_z,patchstar_a = patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar_x}, Y = {patchstar_y}, Z = {patchstar_z}, A = {patchstar_a}")


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
            img_wnd.clear_overlay()
            img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), img_wnd.click_coord, 10, 0, 3))
            print(f'Click Coordinate = {img_wnd.click_coord}')
            img_wnd.click_coord_update()

            img_proc.load_frame(img_wnd.frame)
            img_proc.contour()
            img_wnd.add_overlay(img_proc.img_list[-1].edge)


        #Click once to get a list of centroid points and draw arrowedline from centroid to projected points.

            arrow_pt_list = np.array([])
            centroid_list = np.array([])
            projection_list = np.array([])

            #Take window frame and invert colors for threshold imaging
            blurred_img = cv.blur(img_wnd.frame,(5,5))
            thresh_inv = cv.threshold(blurred_img, 150, 255, cv.THRESH_BINARY_INV)[1]  #150 is the best value 

            thresh_inv_edges = cv.Canny(thresh_inv, 30, 200)
            #cv.imshow('Inverse Image',thresh_inv_edges)
            #cv.waitKey(0)

            inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            #inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            inverse_hierarchy = inverse_hierarchy[0]
            print(f"Contours Hierarchy = {inverse_hierarchy}")

            # Hierarchy = [Next, Previous, First Child, Parent]
            #c[0] = inverse_contours, c[1] = inverse_hierarchy
            #Check if 'Next' location in hierarchy (c[1]) is -1 which is the outer most contour. 
            #All detected contours are not closed so there are no parent and children parameters for these contours. 
            #Locations c[2] and c[3] are set to -1
            outer_contours = [c[0] for c in zip(inverse_contours,inverse_hierarchy) if c[1][0] == -1]

            blank = np.zeros(img_wnd.frame.shape[:2], dtype=np.uint8)
            outer_contours_lines = cv.drawContours(blank, outer_contours, -1, (255,255,255), 1)
            #cv.imshow('Outer Contours Lines',outer_contours_lines)
            #cv.waitKey(0)

            #Must have edge to feed into hough_lines_intersect
            #outer_edge = cv.Canny(outer_contours_lines, 150, 200) #30 or 150 produce similar results.
            outer_edge = cv.Canny(outer_contours_lines, 30, 200)
            #cv.imshow('outer_edge',outer_edge)
            #cv.waitKey(0)

            #Draw arrow from the potential intersection of 2-4 contour lines
            img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(outer_edge)

            #print(img_proc.houghline_p_list)
            #print(img_proc.houghline_intersect_list)

            out_idx = []
            filtered_intersect_list = []

            if img_proc.houghline_p_list is not None:

                for line in img_proc.houghline_p_list:
                    x1,y1,x2,y2 = line[0]
                    img_wnd.frame = cv.line(img_wnd.frame,(x1,y1),(x2,y2),(0,0,0),5)
                    centroid_list = np.append(centroid_list,[x1,y1])
                    centroid_list = np.append(centroid_list,[x2,y2])

                    #Get the index of the projected points that are on or near the HoughlinesP
                    for idx,point in enumerate(img_proc.houghline_intersect_list):
                        x3,y3 = point

                        #Calculate the distance of x3,y3 w.r.t. x1,y1 and x2,y2 of the line to see if it lies between these 2 points
                        check = img_proc.is_between([x1,y1],[x3,y3],[x2,y2])
                        
                        #if x3,y3 lies between these 2 points, save the index
                        if check == True:
                            out_idx = np.append(out_idx,idx)     
                        else:
                            continue
                
                centroid_list = np.reshape(centroid_list,[-1,2])
                centroid_average = np.mean(centroid_list,axis=0)

                arrow_pt_list = np.append(arrow_pt_list,[centroid_average[0],centroid_average[1]])
                #print(f"avg centroid = {arrow_pt_list}")

                #Exclude the projected points on or near the HoughlinesP using the index
                #print("Unique index list:")
                unique_out_idx = list(set(out_idx))
                #print(unique_out_idx)

                #remove duplicates in img_proc.houghline_intersect_list
                filtered_intersect_list = img_proc.houghline_intersect_list.copy()               

                if unique_out_idx is not None:
                    for index in sorted(unique_out_idx, reverse=True):
                        index = int(index)  #index must be integer to pass into ndarray
                        filtered_intersect_list = np.delete(filtered_intersect_list,index,axis=0)

                #print("Houghlines Intersect List:")
                #print(img_proc.houghline_intersect_list)
                #print("Filtered Intersect List:")
                #print(filtered_intersect_list)

            if filtered_intersect_list is not None:
                for point in filtered_intersect_list:
                    pt_x,pt_y = point

                    #x = img_wnd.size[1]
                    #y = img_wnd.size[0]
                    if pt_x > img_wnd.size[1] or pt_y > img_wnd.size[0] or pt_x < 0 or pt_y < 0:
                            continue
                    else:
                        projection_list = np.append(projection_list,[pt_x,pt_y])

                        #Format the projected points to be drawn on the screen
                        #pt_x = int(pt_x)
                        #pt_y = int(pt_y)
                        #dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(0,0,0),thickness=10)

                if not len(projection_list) == 0:
                    projection_list = np.reshape(projection_list,[-1,2])

                    #img_wnd.add_overlay(dots)

                    k_centroid_list,k_sse_list = img_proc.k_means(projection_list)

                    for k_point in k_centroid_list:
                        k_x,k_y = k_point
                        #Format and draw the k-means cluster points on the screen
                        k_x = int(k_x) 
                        k_y = int(k_y)
                        dots = cv.circle(img_wnd.frame,(k_x,k_y),radius=10,color=(255,255,255),thickness=10)

                        img_wnd.add_overlay(dots)

                    k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                    k_centroid_average = np.mean(k_centroid_list,axis=0)

                    #List all of the projected intersection points
                    #print("List of projected points")
                    #print(k_centroid_list)

                    arrow_pt_list = np.append(arrow_pt_list,[k_centroid_average[0],k_centroid_average[1]])          

            #Display arrow and location/angle
            if not len(arrow_pt_list) == 0:
                arrow_pt_list = np.reshape(arrow_pt_list,[-1,2])
                print(f"Arrowed Line points = {arrow_pt_list}")

                start_point = (int(arrow_pt_list[0][0]), int(arrow_pt_list[0][1]))
                end_point = (int(arrow_pt_list[-1][0]), int(arrow_pt_list[-1][1]))
                img_wnd.frame = cv.arrowedLine(img_wnd.frame,start_point,end_point,(0,0,0),thickness=10)
                img_wnd.add_overlay(img_wnd.frame)

                #Calculate angle
                delta_y = int(arrow_pt_list[-1][1]) - int(arrow_pt_list[0][1])
                delta_x = int(arrow_pt_list[-1][0]) - int(arrow_pt_list[0][0])

                angle = np.arctan2(delta_y, delta_x)

                while angle < 0:
                    angle = angle + (2*np.pi)

                #print(f"angle = {angle} ")  #clockwise from start point to end point
                print(f"Clockwise angle = {np.rad2deg(angle)} deg")  #clockwise from start point to end point
                print(f"Counter clockwise angle = {360 - np.rad2deg(angle)} deg") #counter-clockwise from start point to end point 

                print("Direction and Line are calculated with respect to the start coordinate. Imagine a Horizon Line through the Starting Point")
                print(f"start coord = {start_point}")
                print(f"end coord = {end_point}")
                print(f"Move by x = {delta_x}")
                print(f"Move by y = {delta_y}")

                #Rough estimate of Quadrant location based on projected intersection point.
                #screen 1376 is in position 1 and is the X axis
                #screen 1024 is in position 0 and is the Y axis

                center_of_screen = [int(img_wnd.size[1]/2),int(img_wnd.size[0]/2)]
                #print(f"center of screen = {center_of_screen}")
                #print(f"x = {int(img_wnd.size[1])}, y={int(img_wnd.size[0])}")

                #identify the quadrant the end point/tip is located
                arrow_tip_x = int(arrow_pt_list[-1][0])
                arrow_tip_y = int(arrow_pt_list[-1][1])

                if arrow_tip_x > center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                    print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                    print("Quadrant 1 (+x,+y)")
                if arrow_tip_x < center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                    print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                    print("Quadrant 2 (-x,+y)")
                if arrow_tip_x < center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                    print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                    print("Quadrant 3 (-x,-y)")
                if arrow_tip_x > center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                    print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                    print("Quadrant 4 (+x,-y)")

        """
        #Harris Corner detection
            blockSize=2
            kSize=3
            k=3
            img_wnd.frame = cv.cornerHarris(img_wnd.frame, blockSize, kSize, k)
            img_wnd.add_overlay(img_wnd.frame)

            img_wnd.frame[img_wnd.frame > img_wnd.frame.max()] = [255,255,255]
            img_wnd.add_overlay(img_wnd.frame)
            #img_wnd.frame  = cv.dilate(img_wnd.frame, None)
            #h_ret, img_wnd.frame  = cv.threshold(img_wnd.frame, 0.01*img_wnd.frame.max(), 255, 0)
            #img_wnd.frame = np.uint8(img_wnd.frame)
            #img_wnd.add_overlay(img_wnd.frame)


            # find centroids
            #h_ret, h_labels, h_stats, h_centroids = cv.connectedComponentsWithStats(img_wnd.frame)
        
            # define the criteria to stop and refine the corners
            #h_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            #h_corners = cv.cornerSubPix(img_wnd.frame, np.float32(h_centroids), (5,5), (-1,-1), h_criteria)
        
            # Now draw them
            #h_res = np.hstack((h_centroids,h_corners))
            #h_res = np.int0(h_res)
            #print(h_res)
            
            #img_wnd.frame[h_res[:,1],h_res[:,0]] = [0,0,255]
            #img_wnd.frame[h_res[:,3],h_res[:,2]] = [0,255,0]
        """

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