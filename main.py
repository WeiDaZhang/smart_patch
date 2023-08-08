import os
import string
import numpy as np
import cv2 as cv
import threading
import time

class Image_window:
    def __init__(self, name = "Image", size = (1024, 1024)):
        self.name = name
        self.size = size
        self.frame = np.zeros(size)
        self.overlay = np.zeros(size)
        self.click_coord = [0, 0]
        self.previous_click_coord = [0, 0]

    def click_coord_update(self):
        self.previous_click_coord = self.click_coord.copy()

    def get_click_coord(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.click_coord = [x, y]

    def show_live(self):
        while True:
            cv.imshow(self.name, self.frame)
            if cv.waitKey(50) == 27:
                break

    def gen_rand_bw_frame(self):
        self.frame = np.floor(np.random.rand(self.size[0], self.size[1]) * 256).astype(np.uint8)

def main():
    global img
    img = Image_window()

    thr = threading.Thread(target = img.show_live)
    thr.start()

    cv.setMouseCallback(img.name, img.get_click_coord)

    while True:
        img.gen_rand_bw_frame()
        time.sleep(0.1)
        if not img.click_coord == img.previous_click_coord:
            print(img.click_coord)
            img.click_coord_update()

        # Exit if img thread killed
        if not thr.is_alive():
            break

if __name__ == '__main__':
    main()