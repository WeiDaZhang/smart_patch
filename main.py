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
import slicescope as Slicescope
import patchstar as Patchstar
import pvcam as PVCAM
#import condenser as Condenser
from image_window import Image_window

def main():

    print('-----MAIN-----')

    #Global coordinates for the system
    global slicescope_x, slicescope_y, slicescope_z, patchstar_x, patchstar_y, patchstar_z, patchstar_a #, condenser_z


    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')
    
    slicescope_x,slicescope_y,slicescope_z = slicescope.coordinates()
    print(f"Slicescope values X = {slicescope_x}, Y = {slicescope_y}, Z = {slicescope_z}")


    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM5')

    patchstar_x,patchstar_y,patchstar_z,patchstar_a = patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar_x}, Y = {patchstar_y}, Z = {patchstar_z}, A = {patchstar_a}")


    print('-----Camera-----')

    cam = PVCAM(slicescope)

    x_s,y_s,z_s = slicescope.coordinates()
    print(f"x={x_s},y={y_s},z={z_s}")

    x_p,y_p,z_p,a_p = patchstar.coordinates()
    print(f"x={x_p},y={y_p},z={z_p},a={a_p}")

    #patchstar_a = patchstar.approachRelative(patchstar_a,5000_00)
    #cam.image()
    #cv.waitKey(0)
    #patchstar_a = patchstar.approachRelative(patchstar_a,-5000_00)
    #slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,-50_00,0)
    #cam.image()
    #cv.waitKey(0)

    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,240_20,-178_11,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-5_00,0,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-500_00,500_00,0)
    
    print(f"Angle = {patchstar.probe_angle()}")
    #print(f"Probe scale = {patchstar.probe_scale()}")
    #print(f"SS scale = {slicescope.slicescope_scale()}")


    img = cam.image_cross()

    print("Is it correct angle equal to 29?")
    print("   If NO, press ESC to exit program.")
    print("   If YES, press any key to continue.")    
    cv.imshow('Image',img)

    if cv.waitKey(0)==27:  #Press ESC to exit
        sys.exit()


    #Move slicescope or probe away to test autofocus
    #slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,-10_00,0)
    #patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,-50_00,50_00,0)
    #patchstar_a = patchstar.approachRelative(patchstar_a,-1000_00)
    
    #Autofocus
    #slicescope_x,slicescope_y,slicescope_z = cam.autofocus(10_00,400_00)
    
    #Check if contours return a valid value
    tip_coord = cam.contour()
    print(tip_coord)
    #if np.isnan(np.average(tip_coord)):
    #    print("Did not find correct contour value.")
    #    sys.exit()

    img = cam.image_cross()

    print("Is the probe tip around the centre mark?")
    print("   If NO, press ESC to exit program.")
    print("   If YES, press any key to continue.")    
    cv.imshow('Image',img)

    if cv.waitKey(0)==27:  #Press ESC to exit
        sys.exit()


    #This is the image Window, not the Camera, nor the image frame
    global img_wnd
    img_wnd = Image_window()

    img_wnd.set_live_thread()
    img_wnd.thread.start()

    # Wait for image window ready
    img_wnd.wait_window_ready()
    img_wnd.set_mouse_response()

    while True:
        # Generate random pixel map at a rate
        img_wnd.gen_rand_bw_frame()
        time.sleep(0.1)

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img_wnd.click_coord == img_wnd.previous_click_coord:
            img_wnd.clear_overlay()
            img_wnd.overlay = cv.circle(img_wnd.overlay, img_wnd.click_coord, 10, 0, 3)
            print(img_wnd.click_coord)
            img_wnd.click_coord_update()

        # Exit if img_wnd thread killed
        if not img_wnd.thread.is_alive():
            break


    #Movement loop
    slicescope_movement = 100_00
    slicescope_movement_list = np.array([[1,-1],[-2,0],[0,2],[2,0],[-1,-1]])*slicescope_movement
    slicescope_movement_idx = 0

    #Start demo loop
    while True:
        
        #Calibration loop
        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,-5_00)
            tip_coord = cam.contour()
            print(tip_coord)
            if np.isnan(np.average(tip_coord)):
                break
            #cam.calculate_focus_score()
            #cam.image_stop()

        cnt = 0
        while True:
            cnt = cnt + 1
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = cam.contour()
            print(tip_coord)
            if not np.isnan(np.average(tip_coord)):
                break
            #cam.calculate_focus_score()
            print(f"iteration = {cnt}")
            #cam.image_stop()

        tip_coord_list = np.empty(shape=[0,2])
        for idx in range(0,20,1):
            slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,1_00)
            tip_coord = cam.contour()
            print(f"tip = {tip_coord}")
            if not np.isnan(np.average(tip_coord)):
                tip_coord_list = np.append(tip_coord_list,tip_coord,axis=0)
        print(tip_coord_list) # (y values ,x values)

        #cam.image_cross()

        #Move tip to better focus for low magnification objective
        slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,0,0,-5_00)

        #cam.image_cross()

        #Histogram of all the x,y coordinate pixels for tip 
        reson = 10
        centre_coord = np.array([688,512])
        
        # REPLACED W/ hist_top_coord FUNCTION
        #tip_coord_list_x = tip_coord_list[:,1]
        #tip_coord_list_y = tip_coord_list[:,0]
        #tip_coord_list_x_range = np.max(tip_coord_list_x) - np.min(tip_coord_list_x)
        #tip_coord_x_hist, tip_coord_x_edge = np.histogram(tip_coord_list_x,int(tip_coord_list_x_range/reson)+1)
        #bin_idx_x = np.argmax(tip_coord_x_hist)
        
        #tip_coord_list_y_range = np.max(tip_coord_list_y) - np.min(tip_coord_list_y)
        #tip_coord_y_hist, tip_coord_y_edge = np.histogram(tip_coord_list_y,int(tip_coord_list_y_range/reson)+1)
        #bin_idx_y = np.argmax(tip_coord_y_hist)

        #tip_coord_most = np.array([int(np.average([tip_coord_x_edge[bin_idx_x], tip_coord_x_edge[bin_idx_x + 1]])), int(np.average([tip_coord_y_edge[bin_idx_y], tip_coord_y_edge[bin_idx_y + 1]]))])
        #print(f"centre distance = {np.linalg.norm(centre_coord - tip_coord_most)}")

        #print(f"tip coord most = {tip_coord_most}")
        tip_coord_most, *_ = hist_top_coord(np.append(tip_coord_list[:, 1], tip_coord_list[:, 0], axis = 1), reson)


        #Relative coordinate plane rotation/translation for patchstar to slicescope
        #Mechanical movement to pixel translation for patchstar
        pixel_2_patch_scale = 165.45
        pixel_2_patch_angle = np.deg2rad(16.3)  # angle in rad
        #20230721 calibrated
        pixel_2_patch_scale = 163.025
        pixel_2_patch_angle = 0.21146  # angle in rad

        centre_dis = (centre_coord - tip_coord_most)
        print(f"centre distance on pixel = {centre_dis}")
        patch_centre_dis_x = (np.cos(pixel_2_patch_angle)*centre_dis[0] - np.sin(pixel_2_patch_angle)*centre_dis[1])*pixel_2_patch_scale
        patch_centre_dis_y = (np.sin(pixel_2_patch_angle)*centre_dis[0] + np.cos(pixel_2_patch_angle)*centre_dis[1])*pixel_2_patch_scale
        print(f"centre distance on patch = {[patch_centre_dis_x, patch_centre_dis_y]}")
        patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,int(patch_centre_dis_x),int(patch_centre_dis_y),0)
        
        print("______________________________________________________________________________________________")
        print("  Calibration is finished. Press any key to continue.")
        print("______________________________________________________________________________________________")

        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(0)

        #Move tip away from center focused position
        patchstar_a = patchstar.approachRelative(patchstar_a,1000_00)
        
        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)
        
        time.sleep(5)

        #Move slicescope to next coordinate in Movement loop
        slicescope_x,slicescope_y,slicescope_z = slicescope.moveRelative(slicescope_x,slicescope_y,slicescope_z,
                                                                         int(slicescope_movement_list[slicescope_movement_idx,0]),
                                                                         int(slicescope_movement_list[slicescope_movement_idx,1]),0)
        print("------------scope movement----------------")
        print(f"scope move index [0~4] is {slicescope_movement_idx}")
        print(f"scope move x = {int(slicescope_movement_list[slicescope_movement_idx,0])}")
        print(f"scope move y = {int(slicescope_movement_list[slicescope_movement_idx,1])}")

        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)

        time.sleep(5)

        #Mechanical movement to pixel translation for slicescope
        pixel_2_scope_scale = 15.88
        # Stage Calibration 20230721
        pixel_2_scope_scale = 16.3065
        pixel_2_scope_angle = -0.00848

        #Without camera-stage angle correction
        #pixel_movement = slicescope_movement_list[slicescope_movement_idx]/pixel_2_scope_scale
        
        #With camera-stage angle correction
        scope_scaled_movement = slicescope_movement_list[slicescope_movement_idx]/pixel_2_scope_scale
        print(f"non-rotated pixel movement = {scope_scaled_movement}")
        pixel_movement_x = (np.cos(pixel_2_scope_angle)*scope_scaled_movement[0] + np.sin(pixel_2_scope_angle)*scope_scaled_movement[1])
        pixel_movement_y = ( (-1) * np.sin(pixel_2_scope_angle)*scope_scaled_movement[0] + np.cos(pixel_2_scope_angle)*scope_scaled_movement[1])
        pixel_movement = [pixel_movement_x, pixel_movement_y]
        print(f"rotated pixel movement = {pixel_movement}")
        
        patch_centre_dis_x = (np.cos(pixel_2_patch_angle)*pixel_movement[0] - np.sin(pixel_2_patch_angle)*pixel_movement[1])*pixel_2_patch_scale
        patch_centre_dis_y = (np.sin(pixel_2_patch_angle)*pixel_movement[0] + np.cos(pixel_2_patch_angle)*pixel_movement[1])*pixel_2_patch_scale
        print(f"centre distance on patch = {[patch_centre_dis_x, patch_centre_dis_y]}")
        
        #img = cam.image_cross()
        #cv.imshow('Image',img)
        #cv.waitKey(0)

        patchstar_x,patchstar_y,patchstar_z = patchstar.moveRelative(patchstar_x,patchstar_y,patchstar_z,int(patch_centre_dis_x),int(patch_centre_dis_y),0)
        
        img = cam.image_cross()
        cv.imshow('Image',img)
        cv.waitKey(5)

        #Move tip back into camera fov/frame in steps
        for idx in range(0,5,1):
            patchstar_a = patchstar.approachRelative(patchstar_a,-200_00)
            #cam.image_stop()
            img = cam.image_cross()
            cv.imshow('Image',img)
            cv.waitKey(5)
    
        if slicescope_movement_idx < (np.shape(slicescope_movement_list)[0] - 1):
            slicescope_movement_idx = slicescope_movement_idx + 1
        else:
            slicescope_movement_idx = 0
            img = cam.image_cross()
            cv.imshow('Image', img)
            print("This is the end of the while loop. Press any key to continue. Press ESC to exit.")
            if cv.waitKey(10000)==27:  #Press ESC to exit
                break
            print("Loop started. Here we go again!")


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