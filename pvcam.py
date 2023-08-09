#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:23:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import cv2 as cv
import numpy as np

from pyvcam import pvc
from pyvcam.camera import Camera
from pyvcam import constants


class PVCAM:

    # Class parameter

    # Instance method
    def __init__(self,slicescope_instance):
        #Connect to PVCAM
        pvc.init_pvcam()  # Initialize PVCAM 
    
        camera_names = Camera.get_available_camera_names()
        print(camera_names)
        #PMUSBCam00 is the only camera in this list
    
        #self.cam = next(Camera.detect_camera()) # Use generator to find first camera.
        self.cam = Camera.select_camera(camera_names[0])
        self.cam.open()  # Open the camera.

        self.slicescope_instance = slicescope_instance

    def description(self):
        return "Camera"

    def image(self):

        frame = self.cam.get_frame(exp_time=20)
        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)
        #cv.imshow('Image', frame_norm)
        #print(f"row = {len(frame)}, col={len(frame[0])}")
        return frame_norm

    def image_stop(self):

        frame = self.cam.get_frame(exp_time=20)
        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)

        cv.imshow('Image', frame_norm)
        cv.waitKey(0)

        return frame_norm

    def image_cross(self):

        frame = self.cam.get_frame(exp_time=20)

        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)
        #cv.imshow('Image', frame_norm)
        #print(f"row = {len(frame)}, col={len(frame[0])}")

        frame_norm_cross = frame_norm.copy()
        frame_norm_cross[511][686]=0
        frame_norm_cross[511][687]=0
        frame_norm_cross[511][688]=0
        frame_norm_cross[511][689]=0
        frame_norm_cross[511][690]=0

        frame_norm_cross[512][686]=0
        frame_norm_cross[512][687]=0
        frame_norm_cross[512][688]=0
        frame_norm_cross[512][689]=0
        frame_norm_cross[512][690]=0
        
        frame_norm_cross[513][686]=0
        frame_norm_cross[513][687]=0
        frame_norm_cross[513][688]=0
        frame_norm_cross[513][689]=0
        frame_norm_cross[513][690]=0

        frame_norm_cross[510][687]=0
        frame_norm_cross[511][687]=0
        frame_norm_cross[512][687]=0
        frame_norm_cross[513][687]=0
        frame_norm_cross[514][687]=0

        frame_norm_cross[510][688]=0
        frame_norm_cross[511][688]=0
        frame_norm_cross[512][688]=0
        frame_norm_cross[513][688]=0
        frame_norm_cross[514][688]=0

        frame_norm_cross[510][689]=0
        frame_norm_cross[511][689]=0
        frame_norm_cross[512][689]=0
        frame_norm_cross[513][689]=0
        frame_norm_cross[514][689]=0
        #cv.imshow('Image', frame_norm_cross)
        #cv.waitKey(0)

        return frame_norm_cross


    def contour(self):

        input_image = self.image()

        # Find Canny edges
        edged = cv.Canny(input_image, 30, 200)
        #cv.waitKey(0)
  
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
  
        point_list=np.empty(shape=[0,2])

        for contour in contours:
            for point in contour:
                #print(point)
                point_list=np.append(point_list,point,axis=0)
                #print(point_list)
                #cv.waitKey(0)


        print("______")

        #point_std = np.std(point_list,axis=0)
        #print(point_std)

        avg = np.average(point_list, axis=0)
        print(avg)

        if not np.isnan(np.average(point_list)):
            minx_idx = np.argmin(point_list[:, 0])
            print(f"min point inx{minx_idx}")
            minx_idx_coord = point_list[minx_idx]
            print(f"min point coord{minx_idx_coord}")
            print(minx_idx_coord)
            print(point_list)
            copy_input_image = input_image.copy()
            
            copy_input_image[int(minx_idx_coord[1])][int(minx_idx_coord[0])]=0
            tip_coord = np.array([[int(minx_idx_coord[1]), int(minx_idx_coord[0])]])

        # look for farthest point --------------
        #delta_list = np.empty(shape=[0,1])
        #for point in point_list:
        #    delta = point-avg
        #    delta_list = np.append(delta_list,np.linalg.norm(delta))

        #print(sum(delta_list))
        #print(np.isnan(sum(delta_list)))
        #tip_coord = np.empty(shape=[0,2])

        # look for farthest point --------------
        #if not np.isnan(np.average(delta_list)):
        #    delta_idx = np.argmax(delta_list)
        #    delta_idx_coord = point_list[delta_idx]
        #    print(delta_idx_coord)
        #    #print(point_list)
        #    copy_input_image = input_image.copy()
        #    copy_input_image[int(delta_idx_coord[1])][int(delta_idx_coord[0])]=0
        #    tip_coord = np.array([[int(delta_idx_coord[1]), int(delta_idx_coord[0])]])

        #for point in point_list:
            #print(point)
            #if point[0]>=1024:
            #    continue
            #copy_input_image[int(point[1])][int(point[0])]=0
            #copy_input_image[int(delta_idx_coord[1])][int(delta_idx_coord[0])]=0
            #cv.imshow('Copy Input Image',copy_input_image)
            #cv.waitKey(0)
            
        else:
            tip_coord = avg

        print("______")

        #cv.waitKey(0)

        #cv.imshow('Canny Edges After Contouring', edged)
        #cv.waitKey(0)
  
        print("Number of Contours found = " + str(len(contours)))
  
        # Draw all contours
        # -1 signifies drawing all contours
        ##cv.drawContours(input_image, contours, -1, (0, 255, 0), 3)
        #cv.imshow('Contours', input_image)
        #cv.waitKey(0)
        #cv.destroyAllWindows()
        return tip_coord

    def sequence(self,NUM_FRAMES):

        count = 0

        self.cam.start_seq(exp_time=20, num_frames=NUM_FRAMES)
        while count < NUM_FRAMES:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera

            low = np.amin(frame['pixel_data'])  #alias for min
            high = np.amax(frame['pixel_data']) #alias for max
            average = np.average(frame['pixel_data'])

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Rate: {:.1f}\tFrame Count: {:.0f}\n'.format(low, high, average, fps, frame_count))
            count = count + 1

            time.sleep(0.05)

        self.cam.finish()

        # Test basic sequence methods
        frames = self.cam.get_sequence(NUM_FRAMES)
        for frame in frames:
            low = np.amin(frame)
            high = np.amax(frame)
            average = np.average(frame)

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Count: {:.0f}\n'.format(low, high, average, count))
            count = count + 1

        time_list = [i*10 for i in range(1, NUM_FRAMES+1)]
        frames = self.cam.get_vtm_sequence(time_list, constants.EXP_RES_ONE_MILLISEC, NUM_FRAMES)
        for frame in frames:
            low = np.amin(frame)
            high = np.amax(frame)
            average = np.average(frame)

            print('Min:{}\tMax:{}\tAverage:{:.0f}\tFrame Count: {:.0f}\n'.format(low, high, average, count))
            count = count + 1

    def video(self):

        self.cam.start_live(exp_time=20)

        count = 0
        total = 0
        t1 = time.time()
        start = time.time()
        width = 800
        height = int(self.cam.sensor_size[1] * width / self.cam.sensor_size[0])
        dim = (width, height)
        fps = 0

        while True:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera
            frame['pixel_data'] = cv.resize(frame['pixel_data'], dim, interpolation = cv.INTER_AREA)

            high = np.max(frame['pixel_data'])

            cv.imshow('Live', frame['pixel_data']/high)

            if count == 10:
                t1 = time.time() - t1
                fps = 10/t1
                t1 = time.time()
                count = 0

            if cv.waitKey(10) == 27:  #Press ESC to end video feed
                break

            print('Frame Rate: {:.1f}\n'.format(fps))
            count = count + 1
            total = total + 1
        
        self.cam.finish()

    def calculate_focus_score(self):

        #blur = cv.GaussianBlur(self.image(),(3,3),0)
        blur = self.image()
        high = np.max(blur)
        norm = blur/high

        laplacian = cv.Laplacian(norm,cv.CV_64F)
        focus_score = laplacian.var()
        print(focus_score)
        return focus_score

    def autofocus(self,input_stepsize,max_distance):

        local_x,local_y,local_z = self.slicescope_instance.coordinates()

        local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,max_distance/2)
        #Maintain minimum gap between the sample and the 4x objective
        #min_z_abs = -1250000  #max mechanical movement range
        #min_z_abs = -500_00  #Minimum for 4x objective focus.

        best_focus_score = 1
        best_focus_position = 0

        increment = abs(input_stepsize)
        min_increment = 1_00   #1.00 um

        #window is +- max_distance/2 for autofocus
        window_min = local_z - max_distance

        #Number of steps using step size as input
        steps = int(np.ceil( max_distance / increment ))

        for step in range(0,steps,1):

            #Check the z coordinate to ensure slicescope_z does not go beyond minimum z threshold
            z_travel = local_z - increment

            if z_travel >= window_min:
                local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,-increment)
                focus_score = self.calculate_focus_score()

                if focus_score < best_focus_score:
                    best_focus_score = focus_score
                    best_focus_position = local_z 

            else:
                break

        
        #Always try to center the cell or probe tip for the autofocus. This ensures slicescope x,y are fixed. Only difference is relative z.
        local_x,local_y,local_z = self.slicescope_instance.moveRelative(local_x,local_y,local_z,0,0,best_focus_position-local_z)

        #self.image_stop()

        return local_x,local_y,local_z

    def disconnect(self):
    
        self.cam.close()
        pvc.uninit_pvcam()
