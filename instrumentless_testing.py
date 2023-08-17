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

from tip_search_hist import hist_top_coord, get_tip_coord, split_coord_list


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

            # Generate some line segments in frame
            img_wnd.frame = cv.line(np.zeros(shape = img_wnd.size),img_wnd.click_coord, (img_wnd.click_coord[0] + 50, img_wnd.click_coord[1] + 800), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] + 800, img_wnd.click_coord[1] + 50), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] + 5, img_wnd.click_coord[1] - 8), 255, 1, cv.LINE_AA)
            img_wnd.frame = cv.line(img_wnd.frame,img_wnd.click_coord, (img_wnd.click_coord[0] - 7, img_wnd.click_coord[1] + 2), 255, 1, cv.LINE_AA)

            # Fetch the point coordinates in line segments
            img_proc.load_frame(img_wnd.frame)
            img_proc.img_list[-1].edge = np.uint8(img_proc.img_list[-1].frame)
            edge_coord_list = img_proc.frame_2_point_list(img_proc.img_list[-1].edge)
            
            print(split_coord_list(edge_coord_list, 200))
            input()
            # Get polar coordinates
            edge_polar_list = img_proc.cartToPolar(edge_coord_list)
            # Analytical Hough Line Search
            img_proc.hough_2_point_lines(edge_polar_list)
            
            
            # Draw hough space frame
            rhotheta_list, rhotheta_scale, thotheta_offset = img_proc.scale_points_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            hough_wnd.frame = img_proc.point_list_2_frame(rhotheta_list, size = hough_wnd.size)

            # Search Highlight Rho-Thetas
            hough_wnd.frame = img_proc.hist_2d_coord(rhotheta_list, size = hough_wnd.size, resolution = 1)


        # Exit if img_wnd thread killed
        if not img_wnd.thread.is_alive():
            break

if __name__ == '__main__':
    main()