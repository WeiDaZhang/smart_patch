import os
import string
import numpy as np
import cv2 as cv
import threading
import time

class Image_window:
    def __init__(self, name = "Image", size = (1024, 1024), target_frame_rate = 10):
        self.name = name
        self.size = size
        self.frame = np.zeros(size)
        self.overlay = np.ones(size)
        self.click_coord = [0, 0]
        self.previous_click_coord = [0, 0]
        self.target_frame_rate = target_frame_rate

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

