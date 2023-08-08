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
        self.overlay = np.ones(size)
        self.show_frame = np.zeros(size)
        self.click_coord = [0, 0]
        self.previous_click_coord = [0, 0]
        self.target_frame_rate = 10

    def click_coord_update(self):
        self.previous_click_coord = self.click_coord.copy()

    def get_click_coord(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.click_coord = [x, y]

    def show_live(self):
        while True:
            frame_norm = self.frame * self.overlay
            if np.max(frame_norm) > 255:
                frame_norm = frame_norm / np.max(frame_norm)
            frame_norm = np.round(frame_norm * 255).astype(np.uint8)
            cv.imshow(self.name, frame_norm)
            
            # Update at defined frame rate and Wait for ESC to exit
            if cv.waitKey(int(1000/self.target_frame_rate)) == 27:
                break

    def gen_rand_bw_frame(self):
        self.frame = np.floor(np.random.rand(self.size[0], self.size[1]) * 256).astype(np.uint8)

    def clear_overlay(self):
        self.overlay = np.ones(self.size)

def main():
    global img
    img = Image_window()

    thr = threading.Thread(target = img.show_live)
    thr.start()

    # Wait for image window ready
    while not cv.getWindowProperty(img.name, 4):   # WND_PROP_VISIBLE = 4
        time.sleep(0.1)
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