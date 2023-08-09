#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:23:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

# Use # for single line comment
# Use ''' ''' or """ """ as bookends for multiple line comments. Must be indented!

import time
import cv2 as cv
import numpy as np

from pyvcam import pvc
from pyvcam.camera import Camera
from pyvcam import constants


class PVCAM:

    # Class parameter

    # Instance method
    def __init__(self):
        #Connect to PVCAM
        pvc.init_pvcam()  # Initialize PVCAM 
    
        camera_names = Camera.get_available_camera_names()
        print(camera_names)
        #PMUSBCam00 is the only camera in this list
    
        #self.cam = next(Camera.detect_camera()) # Use generator to find first camera.
        self.cam = Camera.select_camera(camera_names[0])
        self.cam.open()  # Open the camera.

    def description(self):
        return "Camera"

    def get_frame(self):

        frame = self.cam.get_frame(exp_time=20)
        high = np.max(frame)
        frame_norm = np.uint8(frame/high * 255)
        return frame_norm

    def get_sequence(self,NUM_FRAMES):

        count = 0

        self.cam.start_seq(exp_time=20, num_frames=NUM_FRAMES)
        while count < NUM_FRAMES:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera
            count = count + 1
            time.sleep(0.05)

        self.cam.finish()

        # Test basic sequence methods
        frames = self.cam.get_sequence(NUM_FRAMES)

        return frames

    def video(self):

        self.cam.start_live(exp_time=20)

        width = 800
        height = int(self.cam.sensor_size[1] * width / self.cam.sensor_size[0])
        dim = (width, height)

        while True:
            frame, fps, frame_count = self.cam.poll_frame()  #cam.poll_frame() is to read in the frames from camera
            frame['pixel_data'] = cv.resize(frame['pixel_data'], dim, interpolation = cv.INTER_AREA)

            high = np.max(frame['pixel_data'])

            cv.imshow('Live', frame['pixel_data']/high)

            if cv.waitKey(10) == 27:  #Press ESC to end video feed
                break

        self.cam.finish()

    def disconnect(self):
    
        self.cam.close()
        pvc.uninit_pvcam()
