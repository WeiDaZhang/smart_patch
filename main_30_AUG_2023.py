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

    arrow_pt_list = np.array([])

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

            edge_coord_list = img_proc.frame_2_point_list(img_proc.img_list[-1].edge)
            print(f'Edge Points Count: {len(edge_coord_list)}')
            
            edge_coord_list_stack = split_index_list(edge_coord_list, 200)
            houghline_rhotheta_list = np.empty(shape = (0, 2))
            for idx in range(edge_coord_list_stack.shape[1]):
                print(edge_coord_list_stack[:, idx])
                # Get polar coordinates
                edge_polar_list = img_proc.cartToPolar(edge_coord_list[edge_coord_list_stack[:, idx], :])
                # Analytical Hough Line Search
                img_proc.hough_2_point_lines(edge_polar_list)
                houghline_rhotheta_list = np.vstack((houghline_rhotheta_list, img_proc.img_list[-1].houghline_rhotheta_list))
            
            
            # Draw hough space frame
            rhotheta_list, rhotheta_scale, thotheta_offset = img_proc.scale_points_2_frame(houghline_rhotheta_list, size = hough_wnd.size)
            hough_wnd.frame = img_proc.point_list_2_frame(rhotheta_list, size = hough_wnd.size)

            #img_proc.contours_2_coord_list()
            #print(f'Coordinates of Contours: {img_proc.img_list[-1].contour_coord_list}')
            #print(f'Average coordinate of Contours: {img_proc.img_list[-1].contour_coord_avg}')
            
            #img_proc.hough_lines()
            #hough_wnd.frame = img_proc.point_list_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            #print(img_proc.img_list[-1].houghline_rhotheta_list)

            #hough_wnd.frame = cv.line(np.zeros(shape = hough_wnd.size),img_wnd.click_coord, (0,hough_wnd.size[0]),255,1,cv.LINE_AA)
            #hough_wnd.frame = cv.line(hough_wnd.frame,img_wnd.click_coord, (hough_wnd.size[1],hough_wnd.size[0]),255,1,cv.LINE_AA)

            #img_proc.load_frame(hough_wnd.frame)
            #img_proc.img_list[-1].edge = np.uint8(img_proc.img_list[-1].frame)
            #print(np.max(img_proc.img_list[-1].edge))
            #img_proc.hough_lines()
            #hough_wnd.frame = img_proc.point_list_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)

            #rhotheta_list, rhotheta_scale, thotheta_offset = img_proc.scale_points_2_frame(img_proc.img_list[-1].houghline_rhotheta_list, size = hough_wnd.size)
            #rhotheta_list = np.int32(rhotheta_list)
            #rhotheta_hists = hist_top_coord(rhotheta_list, resolution = 3)
            #hough_wnd.clear_overlay()
            #hough_wnd.add_overlay(cv.circle(np.ones(hough_wnd.size), np.int32((rhotheta_hists[0][1],rhotheta_hists[0][0])), 10, 0, 3))
            #hough_wnd.add_overlay(cv.circle(np.ones(hough_wnd.size), np.int32(rhotheta_hists[0]), 10, 0, 3))
            #rhotheta_hists[0] = rhotheta_hists[0] / rhotheta_scale + thotheta_offset
            #print(rhotheta_hists)
            
            #for index in range(0,len(img_proc.img_list[-1].hough_line_list)):
            #    cv.line(img_wnd.overlay,(img_proc.img_list[-1].hough_line_list[index][0][0],
            #                         img_proc.img_list[-1].hough_line_list[index][0][1]),
            #                         (img_proc.img_list[-1].hough_line_list[index][0][2],
            #                          img_proc.img_list[-1].hough_line_list[index][0][3]), 0, 1, cv.LINE_AA)

            
        #Click through 3 times to get a list of centroid points and draw arrowedline from first to last.

            #x,y,w,h = cv.boundingRect(img_proc.img_list[-1].edge)
            #img_wnd.frame = cv.rectangle(img_wnd.frame,(x,y),(x+w,y+h),(255,255,255),2)
            #img_wnd.add_overlay(img_wnd.frame)

            #Get the moments of the contours
            M = cv.moments(img_proc.img_list[-1].edge)
            #print(M)

            #Calculate the Centroid from Moments
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            #img_wnd.frame = cv.circle(img_wnd.frame,(cx,cy),radius=5,color=(255,255,255),thickness=5)
            #img_wnd.add_overlay(img_wnd.frame)

            arrow_pt_list = np.append(arrow_pt_list,[cx,cy])
            #print(arrow_pt_list)

            if len(arrow_pt_list)>5:
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
                #plus_x = int(img_wnd.size[1]/2)
                #plus_y = int(img_wnd.size[0]/2)

                print(f"x = {int(img_wnd.size[1])}, y={int(img_wnd.size[0])}")
                print(f"center of screen = {center_of_screen}")
                #print(f"x = {center_of_screen[0]}, y = {center_of_screen[1]}")
                #print(f"x +- {plus_x}")
                #print(f"y +- {plus_y}")

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
                

            #Bitwise invert colors in image
            #inverted_frame = cv.bitwise_not(img_wnd.frame)
            #cv.imshow('inverse',inverted_frame)
            #blurred_img = cv.blur(inverted_frame,(5,5))
            #thresh = cv.threshold(blurred_img, 100, 255, cv.THRESH_BINARY)[1]
            #cv.imshow('thresh',thresh)

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

            edges2 = cv.Canny(drawn_lines, 30, 200)
            cv.imshow('edges 2',edges2)
            cv.waitKey(0)

            #lines = cv.HoughLines(edges2, 1, np.pi / 180, 150, None, 0, 0)
            ## HoughLines returns n x 1 x 2 matrix, reduce (reshape) to n x 2
            #lines_rhotheta_list = np.reshape(lines, (len(lines),2))

            #print(lines_rhotheta_list)

            #if lines is not None:
            #    for i in range(0, len(lines)):
            #        rho = lines[i][0][0]
            #        theta = lines[i][0][1]
            #        a = np.cos(theta)
            #        b = np.sin(theta)
            #        x0 = a * rho
            #        y0 = b * rho
            #        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            #        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            #        draw_hough = cv.line(edges2, pt1, pt2, (255,255,255), 3, cv.LINE_AA)

            #cv.imshow('Draw Hough',draw_hough)
            #cv.waitKey(0)
            
            #Draw arrow from the potential intersection of two contour lines
            img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(edges2)

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

                img_wnd.add_overlay(img_wnd.frame)

        
            if img_proc.houghline_intersect_list is not None:

                for point in img_proc.houghline_intersect_list:
                    pt_x,pt_y = point
                    #print(point)
                    #print(f"x={pt_x}")
                    #print(f"y={pt_y}")

                    #center_coord = (int(1024/2), int(1376/2))
                    #color = (0,0,0)
                    #thickness = 9
                    #image_direction = cv.arrowedLine(img_wnd.frame,point[0],point[1],center_coord,color,thickness)        

                    pt_x = int(pt_x)
                    pt_y = int(pt_y)
                    dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(255,255,255),thickness=10)
                
                img_wnd.add_overlay(dots)    
                #img_wnd.add_overlay(image_direction)
            

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