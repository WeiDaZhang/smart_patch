import os
import string
import numpy as np
import cv2 as cv
import threading
import time

from image_window import Image_window

def main():
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

if __name__ == '__main__':
    main()