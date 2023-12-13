# Dr. Vincent Lee
# tkgui.py
# Description:
# Tkinter GUI classes for GUI frames

from tkinter import *
from PIL import Image, ImageTk

import time
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

from image_window import Image_window
from image_process import Image_process
from camera import Camera
from threading import Thread

class App(Tk):

    def __init__(self, slicescope, patchstar, cam):

        self.slicescope_instance = slicescope
        self.patchstar_instance = patchstar
        self.pvcam_instance = cam

        self.open_flag = True

        self.entry_step = 0
        self.entry_focus_step = 0

        self.k_centroid_list = []
        self.tip_in_focus_corners = []
        self.offset_x_pixels = []
        self.offset_y_pixels = []
        self.tip_angle = []

        self.target1 = np.zeros((3,), dtype = int)
        self.target2 = np.zeros((3,), dtype = int)

        main_frame_color = "white"
        sidebar_color = "gray"
        submenu_color = "blue"

        gui_width = 1800
        gui_height = 1000
        screen_dimensions = str(gui_width) + 'x' + str(gui_height)

        Tk.__init__(self)
        self.title("Bioptix Ltd")
        #self.geometry(screen_dimensions)
        self.resizable(0,0)
        self.bind('<Escape>', lambda e: self.quit())

        # Add menu bar in root window
        menuBar = Menu(self) #Top level menu item
        fileMenuItems = Menu(menuBar, tearoff = 0) #2nd level item (1st below menu item)
        fileMenuItems.add_command(label = 'Quit', command = self.quit) #add quit menu item to this fileMenuItems bar.
        menuBar.add_cascade(label = 'File', menu = fileMenuItems) #top level name and menu attached.
        self.configure(menu = menuBar)

        menu_border = 5

        #Sidebar
        #self.sidebar = Frame(self, bg = sidebar_color, width = 200, height = gui_height, relief = RAISED, borderwidth = menu_border)
        self.sidebar = Frame(self, bg = sidebar_color, relief = RAISED, borderwidth = menu_border)
        self.sidebar.pack(side = LEFT, fill = BOTH, expand = FALSE)

        #Main Frame
        #self.main_frame_width = 1000
        #self.main_frame_height = 1000
        #self.main_frame_border = 0
        #self.main_frame_thickness = 0
        #self.main_frame = Frame(self, bg = main_frame_color, width = self.main_frame_width, height = self.main_frame_height, relief = RAISED, borderwidth = self.main_frame_border, highlightthickness = self.main_frame_thickness)
        #self.main_frame.pack(side = LEFT, fill = BOTH, expand = TRUE)
        #self.main_frame_label = Label(self.main_frame, text = "Main Frame", font = ("Arial",15,"bold"))
        #self.main_frame_label.pack_propagate(FALSE)
        self.main_frame = Frame(self, bg = main_frame_color)
        self.main_frame.pack()
        self.main_frame_label = Label(self.main_frame)
        self.main_frame_label.pack()

        #Logo on top in sidebar
        #self.logo = Frame(self.sidebar, bg = sidebar_color, width = 100, height = 100, relief = RAISED, borderwidth = menu_border)
        self.logo = Frame(self.sidebar, bg = sidebar_color, relief = RAISED, borderwidth = menu_border)
        self.logo.pack()
        self.logo_label = Label(self.logo, text = "Bioptix Ltd", font = ("Arial",15,"bold"), padx=30)
        self.logo_label.pack()

        #Submenus under logo in sidebar
        #self.submenu_frame1 = Frame(self.sidebar, bg = submenu_color, width = 200, height = 400, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame1 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame1.pack()
        self.submenu_frame1_label = Label(self.submenu_frame1, text = "Camera Control", font = ("Arial",15,"bold"), padx=7)
        self.submenu_frame1_label.pack()
        
        #self.submenu_frame2 = Frame(self.sidebar, bg = submenu_color, width = 200, height = 400, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame2 = Frame(self.sidebar, bg = submenu_color, relief = RAISED, borderwidth = menu_border)
        self.submenu_frame2.pack()
        self.submenu_frame2_label = Label(self.submenu_frame2, text = "User Tools", font = ("Arial",15,"bold"), padx=30)
        self.submenu_frame2_label.pack()

        #Buttons

        btn_border = 5
        btn_width = 15
        btn_height = 1
        btn_font = ("Arial",10,"bold")

        btn_open_camera = Button(self.submenu_frame1, text = "OPEN CAM", font = btn_font, width = btn_width, height = btn_height, command = self.open_camera, relief = RAISED, borderwidth = btn_border)
        btn_open_camera.pack()

        btn_start_video = Button(self.submenu_frame1, text = "START VID", font = btn_font, width = btn_width, height = btn_height, command = self.video, relief = RAISED, borderwidth = btn_border)
        btn_start_video.pack()

        btn_stop_video = Button(self.submenu_frame1, text = "STOP VID", font = btn_font, width = btn_width, height = btn_height, command = self.stop_video, relief = RAISED, borderwidth = btn_border)
        btn_stop_video.pack()

        btn_high_mag_box = Button(self.submenu_frame1, text = "High Mag Box", font = btn_font, width = btn_width, height = btn_height, command = self.high_mag_box, relief = RAISED, borderwidth = btn_border)
        btn_high_mag_box.pack()

        btn_video_pop_out = Button(self.submenu_frame1, text = "Vid POP OUT", font = btn_font, width = btn_width, height = btn_height, command = self.video_pop_out, relief = RAISED, borderwidth = btn_border)
        btn_video_pop_out.pack()


        self.label = Label(self.submenu_frame1, text = "Move Camera (um)", font = btn_font)
        self.label.pack()
        self.scale_txt = Entry(self.submenu_frame1, width = 15)
        self.scale_txt.pack()

        btn_set_scale = Button(self.submenu_frame1, text = "Set", command = lambda: self.get_step(self.scale_txt), font = btn_font)
        btn_set_scale.pack()

        if self.entry_step < 100:
            self.entry_step = 0

        btn_left = Button(self.submenu_frame1, text = "LEFT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,-int(self.entry_step),0,0), relief = RAISED, borderwidth = btn_border)
        btn_left.pack()
        btn_right = Button(self.submenu_frame1, text = "RIGHT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,int(self.entry_step),0,0), relief = RAISED, borderwidth = btn_border)
        btn_right.pack()
        btn_up = Button(self.submenu_frame1, text = "UP", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,-int(self.entry_step),0), relief = RAISED, borderwidth = btn_border)
        btn_up.pack()
        btn_down = Button(self.submenu_frame1, text = "DOWN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,int(self.entry_step),0), relief = RAISED, borderwidth = btn_border)
        btn_down.pack()

        btn_zoom_in = Button(self.submenu_frame1, text = "IN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(self.entry_step)), relief = RAISED, borderwidth = btn_border)
        btn_zoom_in.pack()
        btn_zoom_out = Button(self.submenu_frame1, text = "OUT", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(self.entry_step)), relief = RAISED, borderwidth = btn_border)
        btn_zoom_out.pack()


        btn_calibration = Button(self.submenu_frame2, text = "CALibration", font = btn_font, width = btn_width, height = btn_height, command = self.calibration, relief = RAISED, borderwidth = btn_border)
        btn_calibration.pack()

        btn_slicescope_calibration = Button(self.submenu_frame2, text = "Slicescope CAL", font = btn_font, width = btn_width, height = btn_height, command = self.slicescope_calibration, relief = RAISED, borderwidth = btn_border)
        btn_slicescope_calibration.pack()
        btn_patchstar_calibration = Button(self.submenu_frame2, text = "Patchstar CAL", font = btn_font, width = btn_width, height = btn_height, command = self.patchstar_calibration, relief = RAISED, borderwidth = btn_border)
        btn_patchstar_calibration.pack()

        self.focus_label = Label(self.submenu_frame2, text = "Move Focus (um)", font = btn_font)
        self.focus_label.pack()
        self.focus_scale_txt = Entry(self.submenu_frame2, width = 15)
        self.focus_scale_txt.pack()

        btn_focus_set_scale = Button(self.submenu_frame2, text = "Set", command = lambda: self.get_focus_step(self.focus_scale_txt), font = btn_font)
        btn_focus_set_scale.pack()

        if self.entry_focus_step < 100:
            self.entry_focus_step = 100

        btn_focus_down = Button(self.submenu_frame2, text = "Focus DOWN", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.focus_down(int(self.entry_focus_step)), relief = RAISED, borderwidth = btn_border)
        btn_focus_down.pack()

        btn_focus_up = Button(self.submenu_frame2, text = "Focus UP", font = btn_font, width = btn_width, height = btn_height, command = lambda: self.focus_up(int(self.entry_focus_step)), relief = RAISED, borderwidth = btn_border)
        btn_focus_up.pack()

        btn_find_tip = Button(self.submenu_frame2, text = "Find TIP", font = btn_font, width = btn_width, height = btn_height, command = self.find_tip, relief = RAISED, borderwidth = btn_border)
        btn_find_tip.pack()

        btn_autofocus = Button(self.submenu_frame2, text = "AUTOfocus", font = btn_font, width = btn_width, height = btn_height, command = self.autofocus, relief = RAISED, borderwidth = btn_border)
        btn_autofocus.pack()

        btn_harris_corner = Button(self.submenu_frame2, text = "Harris Corner", font = btn_font, width = btn_width, height = btn_height, command = self.harris_corner, relief = RAISED, borderwidth = btn_border)
        btn_harris_corner.pack()

        btn_houghline = Button(self.submenu_frame2, text = "Houghline", font = btn_font, width = btn_width, height = btn_height, command = self.houghline, relief = RAISED, borderwidth = btn_border)
        btn_houghline.pack()

#####----------FUNCTIONS----------#####

    def get_step(self, scale_txt):
        self.entry_step = scale_txt.get()

    def get_focus_step(self, focus_scale_txt):
        self.entry_focus_step = focus_scale_txt.get()

    def get_snapshot(self):

        cap = self.pvcam_instance.get_frame()
        img = Image.fromarray(cap)
        photo_img = ImageTk.PhotoImage(image = img)

        self.main_frame_label.photo_image = photo_img
        self.main_frame_label.configure(image = photo_img)

        return cap

    def open_camera(self):
        self.open_flag = True

    def video(self):

        self.get_snapshot()

        if (self.open_flag == True):
            self.main_frame_label.after(10, self.video)  #repeat same process after every 10 ms

    def stop_video(self):
        self.open_flag = False

    def video_pop_out(self):

        if (self.open_flag == True):
            self.pvcam_instance.video()

    def calibration(self):
        self.slicescope_instance.calibration()
        self.patchstar_instance.calibration()

    def slicescope_calibration(self):
        self.slicescope_instance.calibration()

    def patchstar_calibration(self):
        self.patchstar_instance.calibration()

    def high_mag_box(self):

        cam_control = Camera()
        cap = self.pvcam_instance.get_frame()
        camera_frame = cam_control.box(cap)
        img = Image.fromarray(camera_frame)
        photo_img = ImageTk.PhotoImage(image = img)

        self.main_frame_label.photo_image = photo_img
        self.main_frame_label.configure(image = photo_img)

        if (self.open_flag == True):
            self.main_frame_label.after(10, self.high_mag_box)

    def focus_down(self,step):
        img_proc = Image_process()

        if step < 100:
            step = 100

        #Move slicescope down until probe is out of focus
        cnt = 0
        while True:
            cnt = cnt + 1
            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-int(step))
            
            cap = self.get_snapshot()
            img_proc.load_frame(cap) 
            avg_down,probe_contour = img_proc.contour()  #Get contour (avg,points) 
            print(f"avg_down = {avg_down}")

            if np.isnan(avg_down):
                break

            print(f"iteration = {cnt}")

    def plot_graph(self, ax_handle, x, y):
        ax_handle.plot(x, y, 'k-')
        plt.pause(0.001)

    def focus_up(self, step):
        img_proc = Image_process()

        #Setup figure properties for matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set(title = 'Convergence on the probe tip', xlabel = 'Window number', ylabel = 'Moving average (number of corners)')
        #ax.set(title = 'Convergence on the probe tip', xlabel = 'Iteration', ylabel = 'Number of corners')
        plt.show(block=False)

        if step < 200:
            step = 200

        #Move slicescope up until probe is in focus
        #Calculate rolling average to find the minimum number of corners. This is when the probe is in focus.
        xs = []
        ys = []
        moving_averages = []
        focus_height = []
        focus_corners = []

        thread = Thread(target = self.plot_graph(ax, xs, moving_averages))
        thread.setDaemon(True)
        thread.start()

        cnt = 0
        while True:
            cnt = cnt + 1
            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,int(step))

            cap = self.get_snapshot()
            img_proc.load_frame(cap) 
            avg_up, avg_up_contour = img_proc.contour()
            print(f"avg_up = {avg_up}")

            num_corners, corners = img_proc.harris_corner(cap)
            print(f"Number of corners detected = {num_corners}")

            xs.append(cnt)
            ys.append(num_corners)
            focus_corners.append(corners)
            focus_height.append(self.slicescope_instance.z)
            
            window_size = 5
            
            if len(ys) < window_size:
                moving_averages.append(np.nan)
            else:
                #Calculate the average of current window
                window_average = round(np.sum(ys[-window_size:])/window_size)
                moving_averages.append(window_average)

            #Plot using matplotlib
            #These codes work to constantly update the plot in real time, but is slow.
            #Moving averages real time graph
            self.plot_graph(ax, xs, moving_averages)

            #Harris corners real time graph
            #self.plot_graph(ax,xs,ys)

            if cnt == 100:

                min_convergence_index = moving_averages.index(np.nanmin(moving_averages))
                tip_in_focus_height = focus_height[min_convergence_index]
                tip_in_focus_corners = focus_corners[min_convergence_index]
                self.slicescope_instance.moveAbsolute(self.slicescope_instance.x,self.slicescope_instance.y,int(tip_in_focus_height))

                print(tip_in_focus_height)
                print(tip_in_focus_corners)

                #k_centroid_list,k_sse_list = img_proc.k_means(corners) as reference
                k_centroid_list,k_sse_list = img_proc.k_means(tip_in_focus_corners)
                k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                print("k centroid list")
                print(k_centroid_list)

                self.get_snapshot()

                self.k_centroid_list = k_centroid_list
                self.tip_in_focus_corners = tip_in_focus_corners

                break

            print(f"iteration = {cnt}")

    def find_tip(self):
        img_proc = Image_process()

        #Find probe direction and tip
        img_tip = self.get_snapshot()
        tip_boundary,tip_points = img_proc.detect_probe(img_tip)

        (x1,y1),(x2,y2),tip_angle = img_proc.detect_probe_direction(img_tip, tip_boundary, tip_points)

        #(x2,y2) is your estimated tip coordinate
        if not (np.isnan(x2) or np.isnan(y2)):

            centroid_distance = []

            # Loop over each data point
            for i in range( len(self.k_centroid_list) ):
                x = self.k_centroid_list[i]

                # Calculate distance from each k centroid to estimated probe tip (arrow tip)
                dist = img_proc.compute_L2_distance(x, [x2,y2])
                centroid_distance.append(dist)

            #print(centroid_distance)

            closest_centroid_index =  centroid_distance.index(min(centroid_distance))
            probe_tip = self.k_centroid_list[closest_centroid_index]
            #print("probe tip calculation using min L2 distance")
            #print(probe_tip)

            offset_x_pixels, offset_y_pixels = img_proc.quadrant(img_tip, x2, y2)
            cv.circle(img_tip,(int(probe_tip[0]),int(probe_tip[1])),radius=10,color=(255,255,255),thickness=10)

            # Draw corners
            for corner in range(len(self.tip_in_focus_corners)):
                cv.circle(img_tip, (self.tip_in_focus_corners[corner][0],self.tip_in_focus_corners[corner][1]), 2, (255), 2) 

            cv.imshow('Probe tip', img_tip)
            cv.waitKey(10)

            self.offset_x_pixels = offset_x_pixels
            self.offset_y_pixels = offset_y_pixels

        self.tip_angle = tip_angle


    def autofocus(self):

        if (self.open_flag == True):

            img_proc = Image_process()

            while True:

                cap = self.get_snapshot()
                time.sleep(0.1)

                #Check if the probe is in focus
                img_proc.load_frame(cap)   #Add the window frame to the most recent img_list (img_list[-1]). Pass img_wnd.frame into image_process class for analysis
                check_focus,check_focus_contour = img_proc.contour()  #First contour (avg,points)
                print(f"Is the probe in focus? = {check_focus}")

                if not np.isnan(check_focus):

                    self.focus_down(10_00)

                    self.focus_up(2_00)

                    #Move slicescope and probe up together to remove background interference. Slicescope first, then micromanipulator.
                    common_z_step = min(self.slicescope_instance.z_delta,self.patchstar_instance.z_delta)  #Low magnification reference delta that slicescope and patchstar need to ensure synchronization.
                    self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,common_z_step)
                    self.patchstar_instance.moveRelative(self.patchstar_instance.x,self.patchstar_instance.y,self.patchstar_instance.z,0,0,common_z_step)

                    #Find probe direction and tip
                    self.find_tip()

                    break

                else:
                    print('Probe is OUT OF FOCUS')

            #Move slicescope and probe together back to starting point where probe tip was found. Micromanipulator first, then slicescope.
            self.patchstar_instance.moveRelative(self.patchstar_instance.x,self.patchstar_instance.y,self.patchstar_instance.z,0,0,-common_z_step)
            self.slicescope_instance.moveRelative(self.slicescope_instance.x,self.slicescope_instance.y,self.slicescope_instance.z,0,0,-common_z_step)

    #________________________________________________________________________________

            #Save the coordinates for slicescope and patchstar for later.
            #If user wants to take control of the scope and patchstar now and 
            #then come back to the automation later, these coordinates will help refocus the probe tip. 
            #Assuming patchstar is fixed to table. Only use the controllers to move the x,y,z,a. This will maintain all values relative to each other. 
            self.slicescope_instance.x_tip = self.slicescope_instance.x
            self.slicescope_instance.y_tip = self.slicescope_instance.y
            self.slicescope_instance.z_tip = self.slicescope_instance.z
            self.slicescope_instance.z_tip_dist = common_z_step
            self.patchstar_instance.x_tip = self.patchstar_instance.x
            self.patchstar_instance.y_tip = self.patchstar_instance.y
            self.patchstar_instance.z_tip = self.patchstar_instance.z
            self.patchstar_instance.z_tip_dist = common_z_step
            self.patchstar_instance.a_tip = self.patchstar_instance.a
            self.patchstar_instance.tip_angle = self.tip_angle   #float

            self.patchstar_instance.angle()
            self.patchstar_instance.a_angle = self.patchstar_instance.probe_angle


    #def harris_corner(self, input_image, blockSize=2, kSize=3, k=0.04):
    def harris_corner(self, blockSize=2, kSize=3, k=0.04):

        if (self.open_flag == True):

            img_wnd = Image_window(size = self.pvcam_instance.size)

            img_wnd.set_live_thread()
            img_wnd.thread.setDaemon(True)
            img_wnd.thread.start()

            # Wait for image window ready
            img_wnd.wait_window_ready()
            img_wnd.set_mouse_response()

            img_proc = Image_process()

            while True:
                # Generate random pixel map at a rate
                img_wnd.frame = self.pvcam_instance.get_frame()

                time.sleep(0.1)

                # If clicked a new coordinate, draw circle, and update coordinate
                if not img_wnd.click_coord == img_wnd.previous_click_coord:
                    img_wnd.clear_overlay()
                    img_wnd.click_coord_update()

                    dst = cv.cornerHarris(img_wnd.frame, blockSize, kSize, k)
                    #dst = cv.dilate(dst,None)  #only used to help see the corners when drawn on image.
                    ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
                    dst = np.uint8(dst)

                    # find centroids
                    ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)

                    # define the criteria to stop and refine the corners
                    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
                    corners = cv.cornerSubPix(img_wnd.frame,np.float32(centroids),(5,5),(-1,-1),criteria)

                    corners = np.int0(corners)  #make corners into integer values

                    # Draw corners
                    for corner in range(len(corners)):
                        #cv.circle(img_wnd.frame, (corners[corner][0],corners[corner][1]), 2, (255), 2)
                        img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), (corners[corner][0],corners[corner][1]), 2, 0, 2))   #10, 0, 3 = radius, color, thickness

                    #corners = np.reshape(corners,(-1,2))
                    #cv.imshow('Harris Corner', img_wnd.frame)
                    #cv.waitKey(10)

                # Exit if img_wnd thread killed
                if not img_wnd.thread.is_alive():
                    break


    def houghline(self):

        if (self.open_flag == True):

            img_wnd = Image_window(size = self.pvcam_instance.size)

            img_wnd.set_live_thread()
            img_wnd.thread.setDaemon(True)
            img_wnd.thread.start()

            # Wait for image window ready
            img_wnd.wait_window_ready()
            img_wnd.set_mouse_response()

            img_proc = Image_process()

            while True:
                # Generate random pixel map at a rate
                img_wnd.frame = self.pvcam_instance.get_frame()

                time.sleep(0.1)

                # If clicked a new coordinate, draw circle, and update coordinate
                if not img_wnd.click_coord == img_wnd.previous_click_coord:
                    img_wnd.clear_overlay()
                    #img_wnd.add_overlay(cv.circle(np.ones(img_wnd.size), img_wnd.click_coord, 10, 0, 3))
                    #print(f'Click Coordinate = {img_wnd.click_coord}')
                    img_wnd.click_coord_update()

                    img_proc.load_frame(img_wnd.frame)
                    img_proc.contour()
                    #img_wnd.add_overlay(img_proc.img_list[-1].edge)

                #Click once to get a list of centroid points and draw arrowedline from centroid to projected points.

                    arrow_pt_list = np.array([])
                    centroid_list = np.array([])
                    projection_list = np.array([])

                    #Take window frame and invert colors for threshold imaging
                    blurred_img = cv.blur(img_wnd.frame,(5,5))
                    thresh_inv = cv.threshold(blurred_img, 150, 255, cv.THRESH_BINARY_INV)[1]  #150 is the best value 

                    thresh_inv_edges = cv.Canny(thresh_inv, 30, 200)
                    #cv.imshow('Inverse Image',thresh_inv_edges)
                    #cv.waitKey(0)

                    inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                    #inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                    inverse_hierarchy = inverse_hierarchy[0]
                    print(f"Contours Hierarchy = {inverse_hierarchy}")

                    # Hierarchy = [Next, Previous, First Child, Parent]
                    #c[0] = inverse_contours, c[1] = inverse_hierarchy
                    #Check if 'Next' location in hierarchy (c[1]) is -1 which is the outer most contour. 
                    #All detected contours are not closed so there are no parent and children parameters for these contours. 
                    #Locations c[2] and c[3] are set to -1
                    outer_contours = [c[0] for c in zip(inverse_contours,inverse_hierarchy) if c[1][0] == -1]

                    blank = np.zeros(img_wnd.frame.shape[:2], dtype=np.uint8)
                    outer_contours_lines = cv.drawContours(blank, outer_contours, -1, (255,255,255), 1)
                    #cv.imshow('Outer Contours Lines',outer_contours_lines)
                    #cv.waitKey(0)

                    #Must have edge to feed into hough_lines_intersect
                    #outer_edge = cv.Canny(outer_contours_lines, 150, 200) #30 or 150 produce similar results.
                    outer_edge = cv.Canny(outer_contours_lines, 30, 200)
                    #cv.imshow('outer_edge',outer_edge)
                    #cv.waitKey(0)

                    #Draw arrow from the potential intersection of 2-4 contour lines
                    img_proc.houghline_p_list,img_proc.houghline_intersect_list = img_proc.hough_lines_intersect(outer_edge)

                    #print(img_proc.houghline_p_list)
                    #print(img_proc.houghline_intersect_list)

                    out_idx = []
                    filtered_intersect_list = []

                    if img_proc.houghline_p_list is not None:

                        for line in img_proc.houghline_p_list:
                            x1,y1,x2,y2 = line[0]
                            img_wnd.frame = cv.line(img_wnd.frame,(x1,y1),(x2,y2),(0,0,0),5)
                            centroid_list = np.append(centroid_list,[x1,y1])
                            centroid_list = np.append(centroid_list,[x2,y2])

                            #Get the index of the projected points that are on or near the HoughlinesP
                            for idx,point in enumerate(img_proc.houghline_intersect_list):
                                x3,y3 = point

                                #Calculate the distance of x3,y3 w.r.t. x1,y1 and x2,y2 of the line to see if it lies between these 2 points
                                check = img_proc.is_between([x1,y1],[x3,y3],[x2,y2])
                                
                                #if x3,y3 lies between these 2 points, save the index
                                if check == True:
                                    out_idx = np.append(out_idx,idx)     
                                else:
                                    continue
                        
                        centroid_list = np.reshape(centroid_list,[-1,2])
                        centroid_average = np.mean(centroid_list,axis=0)

                        arrow_pt_list = np.append(arrow_pt_list,[centroid_average[0],centroid_average[1]])
                        #print(f"avg centroid = {arrow_pt_list}")

                        #Exclude the projected points on or near the HoughlinesP using the index
                        #print("Unique index list:")
                        unique_out_idx = list(set(out_idx))
                        #print(unique_out_idx)

                        #remove duplicates in img_proc.houghline_intersect_list
                        filtered_intersect_list = img_proc.houghline_intersect_list.copy()               

                        if unique_out_idx is not None:
                            for index in sorted(unique_out_idx, reverse=True):
                                index = int(index)  #index must be integer to pass into ndarray
                                filtered_intersect_list = np.delete(filtered_intersect_list,index,axis=0)

                        #print("Houghlines Intersect List:")
                        #print(img_proc.houghline_intersect_list)
                        #print("Filtered Intersect List:")
                        #print(filtered_intersect_list)

                    if filtered_intersect_list is not None:
                        for point in filtered_intersect_list:
                            pt_x,pt_y = point

                            #x = img_wnd.size[1]
                            #y = img_wnd.size[0]
                            if pt_x > img_wnd.size[1] or pt_y > img_wnd.size[0] or pt_x < 0 or pt_y < 0:
                                    continue
                            else:
                                projection_list = np.append(projection_list,[pt_x,pt_y])

                                #Format the projected points to be drawn on the screen
                                #pt_x = int(pt_x)
                                #pt_y = int(pt_y)
                                #dots = cv.circle(img_wnd.frame,(pt_x,pt_y),radius=10,color=(0,0,0),thickness=10)

                        if not len(projection_list) == 0:
                            projection_list = np.reshape(projection_list,[-1,2])

                            k_centroid_list,k_sse_list = img_proc.k_means(projection_list)

                            for k_point in k_centroid_list:
                                k_x,k_y = k_point
                                #Format and draw the k-means cluster points on the screen
                                k_x = int(k_x) 
                                k_y = int(k_y)
                                dots = cv.circle(img_wnd.frame,(k_x,k_y),radius=10,color=(255,255,255),thickness=10)

                                img_wnd.add_overlay(dots)

                            k_centroid_list = np.reshape(k_centroid_list,[-1,2])
                            k_centroid_average = np.mean(k_centroid_list,axis=0)

                            #List all of the projected intersection points
                            #print("List of projected points")
                            #print(k_centroid_list)

                            arrow_pt_list = np.append(arrow_pt_list,[k_centroid_average[0],k_centroid_average[1]])          

                    #Display arrow and location/angle
                    if not len(arrow_pt_list) == 0:
                        arrow_pt_list = np.reshape(arrow_pt_list,[-1,2])
                        print(f"Arrowed Line points = {arrow_pt_list}")

                        start_point = (int(arrow_pt_list[0][0]), int(arrow_pt_list[0][1]))
                        end_point = (int(arrow_pt_list[-1][0]), int(arrow_pt_list[-1][1]))
                        img_wnd.frame = cv.arrowedLine(img_wnd.frame,start_point,end_point,(0,0,0),thickness=10)
                        img_wnd.add_overlay(img_wnd.frame)

                        #Calculate angle
                        delta_y = int(arrow_pt_list[-1][1]) - int(arrow_pt_list[0][1])
                        delta_x = int(arrow_pt_list[-1][0]) - int(arrow_pt_list[0][0])

                        angle = np.arctan2(delta_y, delta_x)

                        while angle < 0:
                            angle = angle + (2*np.pi)

                        #print(f"angle = {angle} ")  #clockwise from start point to end point
                        print(f"Clockwise angle = {np.rad2deg(angle)} deg")  #clockwise from start point to end point
                        print(f"Counter clockwise angle = {360 - np.rad2deg(angle)} deg") #counter-clockwise from start point to end point 

                        print("Direction and Line are calculated with respect to the start coordinate. Imagine a Horizon Line through the Starting Point")
                        print(f"start coord = {start_point}")
                        print(f"end coord = {end_point}")
                        print(f"Move by x = {delta_x}")
                        print(f"Move by y = {delta_y}")

                        #Rough estimate of Quadrant location based on projected intersection point.
                        #screen 1376 is in position 1 and is the X axis
                        #screen 1024 is in position 0 and is the Y axis

                        center_of_screen = [int(img_wnd.size[1]/2),int(img_wnd.size[0]/2)]
                        #print(f"center of screen = {center_of_screen}")
                        #print(f"x = {int(img_wnd.size[1])}, y={int(img_wnd.size[0])}")

                        #identify the quadrant the end point/tip is located
                        arrow_tip_x = int(arrow_pt_list[-1][0])
                        arrow_tip_y = int(arrow_pt_list[-1][1])

                        if arrow_tip_x > center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                            print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                            print("Quadrant 1 (+x,+y)")
                        if arrow_tip_x < center_of_screen[0] and arrow_tip_y < center_of_screen[1]:
                            print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                            print("Quadrant 2 (-x,+y)")
                        if arrow_tip_x < center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                            print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                            print("Quadrant 3 (-x,-y)")
                        if arrow_tip_x > center_of_screen[0] and arrow_tip_y > center_of_screen[1]:
                            print(f"arrow tip x = {arrow_tip_x}, arrow tip y = {arrow_tip_y}")
                            print("Quadrant 4 (+x,-y)")