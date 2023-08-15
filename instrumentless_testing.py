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
from image_window import Image_window
from image_process import Image_process

from tip_search_hist import hist_top_coord, get_tip_coord


def main():

    #This is the image Window, not the Camera, nor the image frame
    global img_wnd
    
    canvas_size = [1024, 1376]
    slicescope = []
    
    img_wnd = Image_window(size = canvas_size)

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
        img_wnd.gen_rand_bw_frame()

        time.sleep(0.1)

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img_wnd.click_coord == img_wnd.previous_click_coord:
            img_wnd.clear_overlay()
            cv.circle(img_wnd.overlay, img_wnd.click_coord, 10, 0, 3)
            print(f'Click Coordinate = {img_wnd.click_coord}')
            img_wnd.click_coord_update()

            #img_proc.load_frame(img_wnd.frame)
            #img_proc.contour()
            #img_wnd.add_overlay(img_proc.img_list[-1].edge)

            #img_proc.contours_2_coord_list()
            #print(f'Coordinates of Contours: {img_proc.img_list[-1].contour_coord_list}')
            #print(f'Average coordinate of Contours: {img_proc.img_list[-1].contour_coord_avg}')
            
            #img_proc.hough_lines()
            #hough_wnd.frame = img_proc.point_list_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            #print(img_proc.img_list[-1].houghline_rhotheta_list)

            #img_wnd.frame = cv.line(np.zeros(shape = img_wnd.size),img_wnd.click_coord, (0,img_wnd.size[0]),255,1,cv.LINE_AA)
            #img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.size[1],img_wnd.size[0]),255,1,cv.LINE_AA)

            img_wnd.frame = cv.line(np.zeros(shape = img_wnd.size),img_wnd.click_coord, (img_wnd.click_coord[0] + 50, img_wnd.click_coord[1] + 100), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] + 100, img_wnd.click_coord[1] + 50), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] + 5, img_wnd.click_coord[1] - 8), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] - 7, img_wnd.click_coord[1] + 2), 255, 1, cv.LINE_AA)

            img_proc.load_frame(img_wnd.frame)
            img_proc.img_list[-1].edge = np.uint8(img_proc.img_list[-1].frame)
            edge_point_list = img_proc.frame_2_point_list(img_proc.img_list[-1].edge)
            edge_polar_list = img_proc.cartToPolar(edge_point_list)
            img_proc.hough_2_point_lines(edge_polar_list)
            

            #img_proc.hough_lines()
            rhotheta_list, rhotheta_scale, thotheta_offset = img_proc.scale_points_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            hough_wnd.frame = img_proc.point_list_2_frame(rhotheta_list, size = hough_wnd.size)

            rhotheta_int_list = np.int32(rhotheta_list)
            rhotheta_hist_list = hist_top_coord(rhotheta_int_list, resolution = 8)
            hough_wnd.clear_overlay()
            cv.circle(hough_wnd.overlay, np.int32((rhotheta_hist_list[0][0][1],rhotheta_hist_list[0][0][0])), 10, 2, 3)
            
            print(f'scale{rhotheta_scale}')
            print(f'offset{thotheta_offset}')
            for idx in range(len(rhotheta_hist_list[0])):
                print(f'rho: {rhotheta_hist_list[0][idx][0] / rhotheta_scale[0] + thotheta_offset[0]} theta: {rhotheta_hist_list[0][idx][1] / rhotheta_scale[1] + thotheta_offset[1]} chance: {rhotheta_hist_list[1][idx] * 100} %')
            #hough_wnd.add_overlay(cv.circle(np.ones(hough_wnd.size), np.int32(rhotheta_hists[0]), 10, 0, 3))
            #rhotheta_hists[0] = rhotheta_hists[0] / rhotheta_scale + thotheta_offset
            #print(rhotheta_hists)
            
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