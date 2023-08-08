"""
Read netlist file
Performed on 2022 May 05
Daedalus_Digital_Board_050522.qcv
Daedalus_Digital_Board.txt
"""

import os
import string
import numpy as np
import cv2 as cv
import threading
import time

def img_mouser_click(event, x, y, flags, param):
    global img_click_coord
    if event == cv.EVENT_LBUTTONDOWN:
        img_click_coord = [x, y]

def img_show_live():
    while True:
        cv.imshow(img_window_name, frame)
        if cv.waitKey(50) == 27:
            break

def main():
    img_size = (1024, 1024)
    #--------Global Variable----------
    global img_window_name
    global frame
    global overlap
    global img_click_coord
    img_window_name = 'Image'
    frame = np.zeros(img_size)
    overlap = np.zeros(img_size)
    img_click_coord = [0, 0]
    img_click_coord_last = [0, 0]

    thr = threading.Thread(target = img_show_live)
    thr.start()
    cv.setMouseCallback(img_window_name, img_mouser_click)
    while True:
        frame = np.floor(np.random.rand(img_size[0], img_size[1]) * 256).astype(np.uint8)
        time.sleep(0.1)
        if not img_click_coord == img_click_coord_last:
            print(img_click_coord)
            img_click_coord_last = img_click_coord
        if not thr.is_alive():
            break

if __name__ == '__main__':
    main()