#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri December 1 13:33:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

import cv2 as cv
import time
from pynput import keyboard

#Classes
from slicescope import Slicescope
from patchstar import Patchstar
from pvcam import PVCAM
#from condenser import Condenser
from image_window import Image_window
from image_process import Image_process
from camera import Camera

def main():

    print('-----MAIN-----')

    print('-----Slicescope-----')

    slicescope = Slicescope('COM3')

    slicescope.coordinates()
    print(f"Slicescope values X = {slicescope.x}, Y = {slicescope.y}, Z = {slicescope.z}")

    print('-----Micromanipulator-----')

    patchstar = Patchstar('COM7')

    patchstar.coordinates()
    print(f"Micromanipulator values X = {patchstar.x}, Y = {patchstar.y}, Z = {patchstar.z}, A = {patchstar.a}")

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

    img_proc = Image_process()

    cam_control = Camera(slicescope)

    #Continually detect keyboard inputs using Listener thread. Inputs will join after every key_press.
    with keyboard.Listener(on_press = cam_control.key_press) as listener:

        while True:

            # Image window is constantly updated with cam.get_frame()
            img_wnd.frame = cam.get_frame()

            copy_wnd_frame = img_wnd.frame.copy()

            time.sleep(0.1)

            new_window = cam_control.box(copy_wnd_frame)
            cv.imshow('Camera Control', new_window)
            cv.waitKey(1)

            # Exit if img_wnd thread killed. Press ESC.
            if not img_wnd.thread.is_alive():
                listener.stop()
                #return False
                break

        listener.join() #Must have this inside 'with ... as listener' loop. Inputs will join after every key_press.

    #Close and disconnect all equipment

    cam.disconnect()
    patchstar.close()
    slicescope.close()

    print('-----End of MAIN-----')

if __name__ == '__main__':
    main()