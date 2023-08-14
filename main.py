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

from tip_search_hist import hist_top_coord, get_tip_coord


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
    hough_wnd = Image_window(title = 'Hough Image', size = cam.size)

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
            img_wnd.add_overlay(cv.circle(img_wnd.overlay, img_wnd.click_coord, 10, 0, 3))
            print(f'Click Coordinate = {img_wnd.click_coord}')
            img_wnd.click_coord_update()

            img_proc.load_frame(img_wnd.frame)
            img_proc.contour()
            img_wnd.add_overlay(img_proc.img_list[-1].edge)

            img_proc.contours_2_coord_list()
            print(f'Coordinates of Contours: {img_proc.img_list[-1].contour_coord_list}')
            print(f'Average coordinate of Contours: {img_proc.img_list[-1].contour_coord_avg}')
            
            img_proc.hough_lines()
            hough_wnd.frame = img_proc.point_list_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            print(img_proc.img_list[-1].houghline_rhotheta_list)

            hough_wnd.frame = cv.line(np.zeros(shape = hough_wnd.size),img_wnd.click_coord, (0,hough_wnd.size[1]),255,1,cv.LINE_AA)
            hough_wnd.frame = cv.line(hough_wnd.frame,img_wnd.click_coord, (hough_wnd.size[0],hough_wnd.size[1]),255,1,cv.LINE_AA)

            img_proc.load_frame(hough_wnd.frame)
            img_proc.img_list[-1].edge = np.uint8(img_proc.img_list[-1].frame)
            print(np.max(img_proc.img_list[-1].edge))
            img_proc.hough_lines()
            hough_wnd.frame = img_proc.point_list_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            #for index in range(0,len(img_proc.img_list[-1].hough_line_list)):
            #    cv.line(img_wnd.overlay,(img_proc.img_list[-1].hough_line_list[index][0][0],
            #                         img_proc.img_list[-1].hough_line_list[index][0][1]),
            #                         (img_proc.img_list[-1].hough_line_list[index][0][2],
            #                          img_proc.img_list[-1].hough_line_list[index][0][3]), 0, 1, cv.LINE_AA)



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