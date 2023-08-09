import os
import string
import numpy as np
import cv2 as cv
import threading
import time

from image_window import Image_window

def main():
    global img
    img = Image_window()

    thr = threading.Thread(target = img.show_live)
    thr.start()

    # Wait for image window ready
    while not cv.getWindowProperty(img.name, 4):   # WND_PROP_VISIBLE = 4
        continue
    cv.setMouseCallback(img.name, img.get_click_coord)

    while True:
        # Generate random pixel map at a rate
        img.gen_rand_bw_frame()
        time.sleep(0.1)

        # If clicked a new coordinate, draw circle, and update coordinate
        if not img.click_coord == img.previous_click_coord:
            img.clear_overlay()
            img.overlay = cv.circle(img.overlay, img.click_coord, 10, 0, 3)
            print(img.click_coord)
            img.click_coord_update()

        # Exit if img thread killed
        if not thr.is_alive():
            break

if __name__ == '__main__':
    main()