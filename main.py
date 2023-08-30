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

from tip_search_hist import hist_top_coord, get_tip_coord, split_index_list


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


    global hough_wnd
    hough_wnd = Image_window(title = 'Hough Image', size = (512, 512))

    hough_wnd.set_live_thread()
    hough_wnd.thread.start()

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
            ##thresh_inv = cv.threshold(img_wnd.frame, 100, 255, cv.THRESH_BINARY_INV)[1]
            blurred_img = cv.blur(img_wnd.frame,(5,5))
            thresh_inv = cv.threshold(blurred_img, 90, 255, cv.THRESH_BINARY_INV)[1]

            thresh_inv_edges = cv.Canny(thresh_inv, 30, 200)

            inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            #drawn_lines = cv.drawContours(blurred_img, inverse_contours, 1, (255,255,255),20)
            #cv.imshow('Drawn lines',drawn_lines)
            #cv.waitKey(0)

            blank = np.zeros(img_wnd.frame.shape[:2], dtype=np.uint8)
            drawn_lines = cv.drawContours(blank, inverse_contours, 1, (255,255,255),1)
            #cv.imshow('Drawn lines',drawn_lines)
            #cv.waitKey(0)

            outer_edge = cv.Canny(drawn_lines, 30, 200)
            cv.imshow('outer_edge',outer_edge)
            cv.waitKey(0)
            
            #Draw arrow from the potential intersection of two contour lines
            img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(outer_edge)

            print(img_proc.houghline_p_list)

            if img_proc.houghline_p_list is not None:

                for line in img_proc.houghline_p_list:
                    x1,y1,x2,y2 = line[0]
                    img_wnd.frame = cv.line(img_wnd.frame,(x1,y1),(x2,y2),(255,255,255),5)

                    #Get the moments of the contours
                    M = cv.moments(img_proc.img_list[-1].edge)
                    #print(M)

                    #Calculate the Centroid from Moments
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    centroid_list = np.append(centroid_list,[cx,cy])

                centroid_list = np.reshape(centroid_list,[-1,2])

                img_wnd.add_overlay(img_wnd.frame)

                centroid_average = np.mean(centroid_list,axis=0)
                print(centroid_average)

                arrow_pt_list = np.append(arrow_pt_list,[centroid_average[0],centroid_average[1]])
                print(f"avg centroid = {arrow_pt_list}")

            if img_proc.houghline_intersect_list is not None:

                for point in img_proc.houghline_intersect_list:
                    pt_x,pt_y = point
                    projection_list = np.append(projection_list,[pt_x,pt_y])

                    pt_x = int(pt_x)
                    pt_y = int(pt_y)
                    dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(255,255,255),thickness=10)

                projection_list = np.reshape(projection_list,[-1,2])

                img_wnd.add_overlay(dots)    

                projection_average = np.mean(projection_list,axis=0)

                arrow_pt_list = np.append(arrow_pt_list,[projection_average[0],projection_average[1]])             

            arrow_pt_list = np.reshape(arrow_pt_list,[-1,2])
            start_point = (int(arrow_pt_list[0][0]), int(arrow_pt_list[0][1]))
            end_point = (int(arrow_pt_list[-1][0]), int(arrow_pt_list[-1][1]))
            img_wnd.frame = cv.arrowedLine(img_wnd.frame,start_point,end_point,(255,255,255),thickness=10)
            img_wnd.add_overlay(img_wnd.frame)

            #calculate angle
            delta_y = int(arrow_pt_list[-1][1]) - int(arrow_pt_list[0][1])
            delta_x = int(arrow_pt_list[-1][0]) - int(arrow_pt_list[0][0])

            angle = np.arctan2(delta_y, delta_x)

            while angle < 0:
                angle = angle + (2*np.pi)                    

            print(f"angle = {angle} ")  #clockwise from start point to end point
            print(f"angle = {np.rad2deg(angle)} deg")  #counter-clockwise from start point to end point
            print(f"angle' = {360 - np.rad2deg(angle)} deg")

            print(f"delta x = {delta_x}")
            print(f"delta y = {delta_y}")

            #screen 1376 is in position 1 and is the X axis
            #screen 1024 is in position 0 and is the Y axis

            center_of_screen = [int(img_wnd.size[1]/2),int(img_wnd.size[0]/2)]
            print(f"center of screen = {center_of_screen}")
            print(f"x = {int(img_wnd.size[1])}, y={int(img_wnd.size[0])}")


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