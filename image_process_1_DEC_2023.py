#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed August 9 12:23:00 2023

@author: Dr. Vincent Lee, Dr. WeiDa Zhang
Bioptix Ltd.

"""

import cv2 as cv
import numpy as np
import random
import vispy.scene
from vispy.scene import visuals
import pyautogui

class Image_object:

    # Class parameter

    # Instance method
    def __init__(self, input_image):
        self.frame = input_image
        self.edge = []
        self.contours = []
        self.contour_hierarchy = []
        self.contour_coord_list = np.empty(shape = [0, 2])
        self.contour_coord_avg = []
        self.houghline_p_list = []
        self.houghline_rhotheta_list = []
        self.houghline_intersect_list = []

    def description(self):
        return "Image Object"


class Image_process:

    # Class parameter

    # Instance method
    #def __init__(self, slicescope_instance):
    #    self.slicescope_instance = slicescope_instance
    #    self.img_list = []

    def __init__(self):
        self.img_list = []

    def description(self):
        return "Image Processing Functions"

    def load_frame(self, frame):
        self.img_list.append(Image_object(frame))

    def contour(self):
    #def contour(self,img):
        # Find Canny edges
        self.img_list[-1].edge = cv.Canny(self.img_list[-1].frame, 30, 200)
        #self.img_list[-1].edge = cv.Canny(img, 30, 200)

        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv.findContours(self.img_list[-1].edge, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        print("______")
  
        print("Number of Contours found = " + str(len(contours)))
  
        self.img_list[-1].contours = contours
        self.img_list[-1].contour_hierarchy = hierarchy

        #print(self.img_list[-1].contours)

        point_list=np.empty(shape=[0,2])
        
        for contour in contours:
            for point in contour:
                point_list=np.append(point_list,point,axis=0)

        point_list = np.flip(point_list,1)   #Reverse points list order from [y,x] to [x,y]

        print(point_list)

        avg_contours = np.average(point_list)  #average values in [x,y]

        return avg_contours, point_list

    def get_screenshot(self):
        image = pyautogui.screenshot()
        image = cv.cvtColor(np.array(image),cv.COLOR_RGB2BGR)
        cv.imwrite("capture.png", image)

    def vispy_3D(self,contour_points):
        canvas = vispy.scene.SceneCanvas(keys='interactive', show = True)
        view = canvas.central_widget.add_view()

        scatter = visuals.Markers()
        scatter.set_data(contour_points,edge_width=0,face_color=(1,1,1,0.5),size=5,symbol='o')

        view.add(scatter)

        view.camera = 'turntable'

        axis = visuals.XYZAxis(parent=view.scene)

        vispy.app.run()

    def contours_2_coord_list(self):
        # Squash contour to a list of coordinates
        for contour in self.img_list[-1].contours:
            for point in contour:
                self.img_list[-1].contour_coord_list = np.append(self.img_list[-1].contour_coord_list, point, axis = 0)

        self.img_list[-1].contour_coord_avg = np.average(self.img_list[-1].contour_coord_list, axis = 0)

    def hough_lines(self, rho = 1, theta = np.pi / 180 / 10, min_votes = 1):
        #self.img_list[-1].hough_line_list = cv.HoughLinesP(self.img_list[-1].edge, rho, theta, min_votes, None, 150, 50)
        hough_line_list = cv.HoughLines(self.img_list[-1].edge, rho, theta, min_votes, None, 100, 100)
        # HoughLines returns n x 1 x 2 matrix, reduce (reshape) to n x 2
        self.img_list[-1].houghline_rhotheta_list = np.reshape(hough_line_list, (len(hough_line_list),2))

    def hough_lines_intersect(self, input_edges, rho=1, theta = np.pi / 180, line_vote_threshold = 200, min_line_length = 200, max_line_gap = 20):

        linesP = cv.HoughLinesP(input_edges, rho, theta, line_vote_threshold, None, min_line_length, max_line_gap)

        #print("Houghlines P:")
        #print(linesP)
        #print(np.size(linesP))  #same
        #print(linesP.size)      #same
        #print(np.size(linesP[0]))  
        #print(linesP[0].size)      #this is taking the ndarray and using the '.size' property

        #linesP output is an ndarray that has n times 1x4 vectors. 1 ndarray = [ [[vector 1 (1x4)]]  [[vector 2 (1x4)]] ... ] 

        P_list = []

        if linesP is not None:

            #initialize index for enumerate()
            idx=0

            num_lines = int(linesP.size/linesP[0].size)
            print(f"num lines = {num_lines}")
            range_num_lines = range(0,num_lines-1)   #The total number of lines to compare minus one. The last line has no comparison.

            #iterate over row dimension np.size(linesP[0]) == linesP[0].size
            for idx, j in enumerate(range_num_lines):
                for k in range_num_lines[idx:]:
                    if k > idx:
                        x1,y1,x2,y2 = linesP[j][0]
                        x3,y3,x4,y4 = linesP[k][0]
                        #As noted above, the total number of lines to compare minus one. The last line has no comparison anyway.

                        #print out all of the lines that will be compared.
                        #print(linesP[j][0])
                        #print(linesP[k][0])

                        #Lines L1 and L2 
                        # L1: (x1,y1) and (x2,y2)
                        # L2: (x3,y3) and (x4,y4)
            
                        denom = ((x1-x2)*(y3-y4)) - ((y1-y2)*(x3-x4))

                        if not denom == 0:
                            Px_numer = (((x1*y2)-(y1*x2))*(x3-x4)) - ((x1-x2)*((x3*y4)-(y3*x4)))
                            Py_numer = (((x1*y2)-(y1*x2))*(y3-y4)) - ((y1-y2)*((x3*y4)-(y3*x4)))

                            Px = int(Px_numer/denom)
                            Py = int(Py_numer/denom)

                            #Ensure all intersections are positive and within the camera coordinate system
                            if not ((Px < 0) or (Py < 0)):
                                P_list = np.append(P_list, [Px, Py], axis = 0)

        P_list = np.reshape(P_list,(-1,2))
        P_list.astype(int)
        #print(P_list)

        return linesP,P_list

    def distance(self,a,b):
        #Calculate the distance between 2 coordinates
        #x1,y1 = a[0] a[1]
        #x2,y2 = b[0] b[1]
        
        dist = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        return dist 

    def is_between(self,a,c,b):

        #check if self.distance(a,c) + self.distance(c,b) == self.distance(a,b)
        tolerance = 100  # 100 is good
        check = self.distance(a,c) + self.distance(c,b)
        
        if check < self.distance(a,b)+tolerance:
            return True
        else:
            return False

    def scale_points_2_frame(self, point_list, size = (1024, 1024)):
        scale = [1, 1]
        offset = [np.min(point_list[:, 0]), np.min(point_list[:, 1])]
        point_list[:, 0] = point_list[:, 0] - np.min(point_list[:, 0])
        point_list[:, 1] = point_list[:, 1] - np.min(point_list[:, 1])
        scale[0] = (size[0] - 1) / np.max(point_list[:, 0])
        scale[1] = (size[1] - 1) / np.max(point_list[:, 1])
        point_list[:, 0] = point_list[:, 0] * scale[0]
        point_list[:, 1] = point_list[:, 1] * scale[1]
        return point_list, scale, offset

    def point_list_2_frame(self, point_list, size = (1024, 1024)):
        frame = np.zeros(shape = size)
        for idx in range(0, len(point_list)):
            frame[np.floor(point_list[idx, 0]).astype(int), np.floor(point_list[idx, 1]).astype(int)] += 1
        return frame

    def frame_2_point_list(self, frame, contrast = 0.5):
        frame = frame - np.min(frame)
        frame = frame / np.max(frame)
        point_list = np.empty(shape = (0, 2))
        for idx_row in range(0, frame.shape[0]):
            for idx_col in range(0, frame.shape[1]):
                if frame[idx_row, idx_col] > contrast:
                    point_list = np.append(point_list, [[idx_row, idx_col]], axis = 0)
        return point_list

    def cartToPolar(self, coord_list, angleInDegress = False):
        mag = np.sqrt(np.square(coord_list[:, 0]) + np.square(coord_list[:, 1]))
        # matrix-row = y
        # matrix-col = x
        ang = np.arctan2(coord_list[:, 0], coord_list[:, 1])
        if angleInDegress:
            ang = ang / np.pi * 180 
        return np.column_stack((mag.T, ang.T))

    def hough_2_point_lines(self, rho_theta_list):
        sin_minus_theta_list = np.sin(-rho_theta_list[:, 1])
        cos_minus_theta_list = np.cos(-rho_theta_list[:, 1])
        rho_sin_product_list = rho_theta_list[:, 0] * sin_minus_theta_list
        rho_cos_product_list = rho_theta_list[:, 0] * cos_minus_theta_list
        combination_idx_list = np.stack(np.triu_indices(len(rho_theta_list), k=1), axis=-1)
        actan_nominator = np.diff(rho_sin_product_list[combination_idx_list], axis = 1)
        actan_denominator = np.diff(rho_cos_product_list[combination_idx_list], axis = 1)
        actan_nonzero_idx_list = np.flatnonzero(actan_denominator)
        cross_theta = np.pi/2 - np.arctan(actan_nominator[actan_nonzero_idx_list] / actan_denominator[actan_nonzero_idx_list])
        rho_theta_combination_list = rho_theta_list[combination_idx_list[actan_nonzero_idx_list, 0], :]
        cross_rho = rho_theta_combination_list[:, 0] * np.cos(cross_theta[:, 0] - rho_theta_combination_list[:, 1])
        cross_rho.shape = (cross_rho.size, 1)
        self.img_list[-1].houghline_rhotheta_list = np.column_stack((cross_rho, cross_theta))
        
        #rho_theta_2_point_list = combinations(rho_theta_list, 2)
        #cross_rho_theta_list = np.empty(shape = (0, 2))
        #for rho_theta_2_point in list(rho_theta_2_point_list):
        #    #input(rho_theta_2_point)
        #    actan_nominator = rho_theta_2_point[0][0]*np.sin(-rho_theta_2_point[0][1]) - rho_theta_2_point[1][0]*np.sin(-rho_theta_2_point[1][1])
        #    actan_denominator = rho_theta_2_point[0][0]*np.cos(-rho_theta_2_point[0][1]) - rho_theta_2_point[1][0]*np.cos(-rho_theta_2_point[1][1])
        #    if actan_denominator == 0:
        #        continue
        #    cross_theta = np.pi/2 - np.arctan(actan_nominator / actan_denominator)
        #    cross_rho = rho_theta_2_point[0][0]*np.cos(cross_theta - rho_theta_2_point[0][1])
        #    cross_rho_theta_list = np.append(cross_rho_theta_list, [[cross_rho, cross_theta]], axis = 0)
        #    #print([cross_rho, cross_theta])
        #    #if len(cross_rho_theta_list) > 10:
        #    #    break
        #self.img_list[-1].houghline_rhotheta_list = cross_rho_theta_list
        #return cross_rho_theta_list

    def hist_2d_coord(self, coord_list, size = (1024, 1024), resolution = 10):
        coord_list = np.array(coord_list)
        print(f"Coordinate List: \n{coord_list}")
        
        hist_bins = np.array([np.ceil(size[0]/resolution), np.ceil(size[1]/resolution)]).astype(int)
        # Histogram2d input: x first, y second, output: row = x, col = y
        coord_hist_T, coord_edge_x, coord_edge_y = np.histogram2d(coord_list[:, 1],   # second column in coorinate list is x
                                                                coord_list[:, 0],   # first column in coorinate list is y
                                                                hist_bins)
        #print(f"hist: {coord_hist}")
        #print(f"edge x: {coord_edge_x}")
        #print(f"edge y: {coord_edge_y}")
            
        return coord_hist_T.T

    #K-means Cluster method
    def k_means(self, data):

        if len(data) == 0:
            return None
        else:
            k_centroids = []

            # Sample initial centroids of projected intersections
            random_indices = random.sample(range( data.shape[0] ), len(data))
            for i in random_indices:
                k_centroids.append(data[i])

            #print(k_centroids)

            # Create a list to store which centroid is assigned to each dataset
            assigned_k_centroids = [0]*len(data)

            # Number of dimensions in centroid
            num_centroid_dims = data.shape[1]

            # List to store SSE for each iteration 
            sse_list = []

            # Loop over iterations
            for n in range( len(data) ):

                # Loop over each data point
                for i in range( len(data) ):
                    x = data[i]

                    # Get the closest centroid
                    closest_centroid = self.get_closest_centroid(x, k_centroids)
        
                    # Assign the centroid to the data point.
                    assigned_k_centroids[i] = closest_centroid

                # Loop over k_centroids and compute the new ones.
                for c in range( len(k_centroids) ):
                    # Get all the data points belonging to a particular cluster
                    cluster_data = [data[i] for i in range( len(data) ) if assigned_k_centroids[i] == c]
    
                    if cluster_data == []:
                        continue
                    else:
                        # Initialise the list to hold the new centroid
                        new_k_centroid = [0]*len(k_centroids[0])
        
                        # Compute the average of cluster members to compute new centroid
                        # Loop over dimensions of data
                        for dim in range( num_centroid_dims ):
                            dim_sum = [ x[dim] for x in cluster_data ]
                            dim_sum = sum(dim_sum) / len(dim_sum)
                            new_k_centroid[dim] = dim_sum

                    # assign the new centroid
                    k_centroids[c] = new_k_centroid

                # Compute the SSE for the iteration
                sse = self.compute_sse(data, k_centroids, assigned_k_centroids)
                sse_list.append(sse)

        return k_centroids,sse_list

    def compute_L2_distance(self, x, centroid):
        dist = 0

        # Loop over the dimensions. Take squared difference and add to dist 
        for i in range(len(x)):
            dist = dist + ( centroid[i] - x[i] )**2

        return dist

    def get_closest_centroid(self, x, centroids):
        
        centroid_distances = []

        # Loop over each centroid and calculate the distance from each data point.
        for centroid in centroids:
            dist = self.compute_L2_distance(x, centroid)
            centroid_distances.append(dist)

        # Get the index of the centroid with the smallest distance to each data point 
        closest_centroid_index =  min(range(len(centroid_distances)), key=lambda x: centroid_distances[x])

        return closest_centroid_index
    
    def compute_sse(self, data, k_centroids, assigned_k_centroids): 
        #Sum of Squared Errors (sse). Need to look for the elbow point to get the optimum number of iterations.
        sse = 0
    
        if len(data) == 0:
            return None
        else:
            # Compute the squared distance for each data point and add. 
            for i,x in enumerate(data):
    	        # Get the associated centroid for data point
                centroid = k_centroids[assigned_k_centroids[i]]

                # Compute the distance to the centroid
                dist = self.compute_L2_distance(x, centroid)

                # Add to the total distance
                sse = sse + dist

            sse = sse / len(data)

        return sse

    def harris_corner(self, input_image, blockSize=2, kSize=3, k=0.04):

        dst = cv.cornerHarris(input_image, blockSize, kSize, k)
        #dst = cv.dilate(dst,None)  #only used to help see the corners when drawn on image.
        ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
        dst = np.uint8(dst)

        # find centroids
        ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)

        # define the criteria to stop and refine the corners
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        corners = cv.cornerSubPix(input_image,np.float32(centroids),(5,5),(-1,-1),criteria)

        corners = np.int0(corners)  #make corners into integer values

        # Draw corners
        #for corner in range(len(corners)):
        #    cv.circle(input_image, (corners[corner][0],corners[corner][1]), 2, (255), 2) 

        corners = np.reshape(corners,(-1,2))

        return len(corners),corners   #return the total number of corners coordinates

    def detect_probe(self, img_capture):

        #pass in image window img_wnd.frame

        height,width = img_capture.shape

        # [0] = 1024, [1] = 1376
        #print(height) #[0]
        #print(width)  #[1]

        thresh_inv = cv.threshold(img_capture, 150, 255, cv.THRESH_BINARY_INV+cv.THRESH_OTSU)[1]  #150 is the best value
        #cv.imshow('Threshold Image',thresh_inv)
        #cv.waitKey()

        inverse_contours, inverse_hierarchy = cv.findContours(thresh_inv, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Kernel structure
        # Increase the kernel size to blend the thresholded images more.
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(9,9))

        # Morphological techniques
        # erosion followed by dilation
        closing_img = cv.morphologyEx(thresh_inv, cv.MORPH_CLOSE, kernel)
        #cv.imshow('Closing Image', closing_img)
        #cv.waitKey()

        inverse_hierarchy = inverse_hierarchy[0]
        #print(f"Contours Hierarchy = {inverse_hierarchy}")

        hull_list = []

        closing_cnts, closing_cnts_hierarchy = cv.findContours(closing_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        tmp=0
        for c in range(len(closing_cnts)):
             
            hull = cv.convexHull(closing_cnts[c])
                
            if len(hull)>tmp:
                tmp=len(hull)
                hull_list.append(hull)

        #print(hull_list)

        cv.drawContours(img_capture, hull_list, -1, (255), thickness=3)
        #cv.imshow('Convex Hull Fill',img_capture)
        #cv.waitKey()

        cv.fillPoly(img_capture, hull_list, (255))
        #cv.imshow('Convex Hull Fill Poly',img_capture)
        #cv.waitKey() 

        #gauss_blur = cv.GaussianBlur(img_wnd.frame,(5,5),0)
        #hull_thresh = cv.threshold(gauss_blur, 150, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)[1]  #150 is the best value
        hull_thresh = cv.threshold(img_capture, 150, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)[1]  #150 is the best value
        #cv.imshow('Hull Threshold',hull_thresh)
        #cv.waitKey()

        bounding_cnts, bounding_cnts_hierarchy = cv.findContours(hull_thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        #print(bounding_cnts)

        #Only keep the maximum rectangle and ellipse 
        tmp=0
        #maxRect = []
        maxEllipse = []
        points = np.zeros((4,2), dtype=int)

        # Find the rotated rectangles and ellipses for each contour
        minRect = [None]*len(bounding_cnts)
        minEllipse = [None]*len(bounding_cnts)

        for i, c in enumerate(bounding_cnts):

            minRect[i] = cv.minAreaRect(c)
            #[(xc, yc), (width, height), angle] = returned minAreaRect values
            #[(x center, y center), (width, height), theta angle] = returned minAreaRect values

            if c.shape[0] > 5:
                minEllipse[i] = cv.fitEllipse(c)
                #[(xc, yc), (width, height), angle] = returned fitEllipse values
                #[(x center, y center), (major semi-axis, minor semi-axis), theta angle] = returned fitEllipse values

                if not minEllipse[i] == None:
                    #Get min rectangle points
                    box = cv.boxPoints(minRect[i])

                    #Keep coordinates positive and within camera frame
                    for k in range(len(box)):
                        if box[k][0] < 0:
                            box[k][0] = 0

                        if box[k][1] < 0:
                            box[k][1] = 0

                        if box[k][0] > int(width):
                            box[k][0] = int(width)

                        if box[k][1] > int(height):
                            box[k][1] = int(height)

                    #Shoelace formula to calculate area of Polygon
                    area=0
                    for k in range(len(box)):
                        j = (k + 1) % len(box)
                        area += box[k][0] * box[j][1]
                        area -= box[j][0] * box[k][1]
                        
                    area = abs(area) / 2
                    #print(f'PolyArea = {area}')

                    screen_size = int(width)*int(height)
                    if area>tmp and area<(screen_size/2):
                        tmp=area
                        #maxRect = minRect[i]
                        maxEllipse = minEllipse[i]
                        points[0][0] = box[0][0]
                        points[0][1] = box[0][1]
                        points[1][0] = box[1][0]
                        points[1][1] = box[1][1]
                        points[2][0] = box[2][0]
                        points[2][1] = box[2][1]
                        points[3][0] = box[3][0]
                        points[3][1] = box[3][1]

        #print(points)
        #print(maxEllipse)

        return maxEllipse, points 
    
    def detect_probe_direction(self, img_capture, maxEllipse, points):
        
        #pass in image window img_wnd.frame

        height,width = img_capture.shape

        # [0] = 1024, [1] = 1376
        #print(height) #[0]
        #print(width)  #[1]
        
        ## reference: print(int(img_wnd.size[1]),int(img_wnd.size[0]))
        center_of_screen = [int(width/2),int(height/2)]
        q1_x = int(center_of_screen[0] + (center_of_screen[0]/4))
        q1_y = int(center_of_screen[1] - (center_of_screen[1]/4))
        q2_x = int(center_of_screen[0] - (center_of_screen[0]/4))
        q2_y = int(center_of_screen[1] - (center_of_screen[1]/4))
        q3_x = int(center_of_screen[0] - (center_of_screen[0]/4))
        q3_y = int(center_of_screen[1] + (center_of_screen[1]/4))
        q4_x = int(center_of_screen[0] + (center_of_screen[0]/4))
        q4_y = int(center_of_screen[1] + (center_of_screen[1]/4))

        font = cv.FONT_HERSHEY_SIMPLEX 
        fontScale = 1
        colorText = (0,0,0)
        thickness = 2

        #pass in image window img_wnd, largest Ellipse parameters, rectangle box points

        if maxEllipse[2] > 90:
            theta = abs(180 - maxEllipse[2])
        else:
            theta = abs(maxEllipse[2])

        if maxEllipse[1][0] < maxEllipse[1][1]:            
            major_axis = maxEllipse[1][1]
        else:
            major_axis = maxEllipse[1][0]

        x_move = int((major_axis/2) * np.sin(np.pi*(theta/180)))
        y_move = int((major_axis/2) * np.cos(np.pi*(theta/180)))

        #Check if final bounding box touches edges. 
        #Use the angle and edges/ellipse center and major axis to estimate which quadrant the probe tip is in.
        edge_points = []
        border = 10 #pixels
        
        for k in range(len(points)):

            if (points[k][0] <= border) or (points[k][1] <= border) or (points[k][0] >= (int(width)-border)) or (points[k][1] >= (int(height)-border)):
            #At least one of the Box coordinates is near an edge

                edge_points = np.append(edge_points, [points[k][0],points[k][1]])

        if not len(edge_points) == 0: #if the edge_points list is NOT empty
            #Check intersection at edge of screen and use the angle to determine the starting quadrant
            # maxEllipse[2] is the angle in degrees

            edge_points = np.reshape(edge_points,[-1,2])
            #print(edge_points)

            #Check left side of screen
            #If the x coordinate of the 2 points in edge_points are 0 -> Q2 or Q3
            if edge_points[0][0] == 0 or edge_points[1][0] == 0:
                    
                    #If the Ellipse center coordinate is beyond the left side of the screen
                if maxEllipse[0][0] < edge_points[0][0] or maxEllipse[0][0] < edge_points[1][0]:
                    x1 = int((edge_points[0][0] + edge_points[1][0])/2)
                    y1 = int((edge_points[0][1] + edge_points[1][1])/2)
                else:
                    x1 = int(maxEllipse[0][0])
                    y1 = int(maxEllipse[0][1])

                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q2
                    if edge_points[0][1] < (int(height)/2) or edge_points[1][1] < (int(height)/2):
                        print("Q2 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q2 = ' + '{:.2f}'.format(maxEllipse[2]), (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q3
                    if edge_points[0][1] > (int(height)/2) or edge_points[1][1] > (int(height)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q2
                    if edge_points[0][1] < (int(height)/2) or edge_points[1][1] < (int(height)/2):
                        print("Q2 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q2 = ' + '{:.2f}'.format(maxEllipse[2]), (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q3
                    if edge_points[0][1] > (int(height)/2) or edge_points[1][1] > (int(height)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check


                if int(maxEllipse[2]) == 90:
                    #Q2
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][1] < (int(height)/2):
                        print("Q2 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q2 = ' + '{:.2f}'.format(maxEllipse[2]), (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q3
                    if edge_points[0][1] > (int(height)/2) and edge_points[1][1] > (int(height)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Between Q2 and Q3
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][1] > (int(height)/2):
                        print("NOT Q2 or Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q2 or Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

                    if edge_points[0][1] > (int(height)/2) and edge_points[1][1] < (int(height)/2):
                        print("NOT Q2 or Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q2 or Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

            #Check right side of screen
            #If the x coordinate of the 2 points in edge_points are max at the screen edge -> Q1 or Q4
            if edge_points[0][0] == int(width) or edge_points[1][0] == int(width):
                    
                #If the Ellipse center coordinate is beyond the left side of the screen
                if maxEllipse[0][0] > edge_points[0][0] or maxEllipse[0][0] > edge_points[1][0]:
                    x1 = int((edge_points[0][0] + edge_points[1][0])/2)
                    y1 = int((edge_points[0][1] + edge_points[1][1])/2)
                else:
                    x1 = int(maxEllipse[0][0])
                    y1 = int(maxEllipse[0][1])

                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q1
                    if edge_points[0][1] < (int(height)/2) or edge_points[1][1] < (int(height)/2):
                        print("Q1 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q1 = ' + '{:.2f}'.format(maxEllipse[2]), (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q4
                    if edge_points[0][1] > (int(height)/2) or edge_points[1][1] > (int(height)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q1
                    if edge_points[0][1] < (int(height)/2) or edge_points[1][1] < (int(height)/2):
                        print("Q1 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q1 = ' + '{:.2f}'.format(maxEllipse[2]), (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 + y_move

                        if x_check < 0 or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q4
                    if edge_points[0][1] > (int(height)/2) or edge_points[1][1] > (int(height)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 + y_move

                        if x_check < 0 or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                if int(maxEllipse[2]) == 90:
                    #Q1
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][1] < (int(height)/2):
                        print("Q1 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q1 = ' + '{:.2f}'.format(maxEllipse[2]), (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = x1 - x_move
                        y2 = y1 - y_move

                        if x_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q4
                    if edge_points[0][1] > (int(height)/2) and edge_points[1][1] > (int(height)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = x1 - x_move
                        y2 = y1 - y_move

                        if x_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Between Q1 and Q4
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][1] > (int(height)/2):
                        print("NOT Q1 or Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q1 or Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

                    if edge_points[0][1] > (int(height)/2) and edge_points[1][1] < (int(height)/2):
                        print("NOT Q1 or Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q1 or Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

            #Check bottom of screen
            #If the y coordinate of the 2 points in edge_points are max at the screen edge -> Q3 or Q4
            if edge_points[0][1] == int(height) or edge_points[1][1] == int(height):
                    
                #If the Ellipse center coordinate is beyond the left side of the screen
                if maxEllipse[0][1] > edge_points[0][1] or maxEllipse[0][1] > edge_points[1][1]:
                    x1 = int((edge_points[0][0] + edge_points[1][0])/2)
                    y1 = int((edge_points[0][1] + edge_points[1][1])/2)
                else:
                    x1 = int(maxEllipse[0][0])
                    y1 = int(maxEllipse[0][1])

                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q3
                    if edge_points[0][0] < (int(width)/2) or edge_points[1][0] < (int(width)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check
                    
                    #Q4
                    if edge_points[0][0] > (int(width)/2) or edge_points[1][0] > (int(width)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q3
                    if edge_points[0][0] < (int(width)/2) or edge_points[1][0] < (int(width)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q4
                    if edge_points[0][0] > (int(width)/2) or edge_points[1][0] > (int(width)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                if int(maxEllipse[2]) == 0 or int(maxEllipse[2]) == 180:
                    #Q3
                    if edge_points[0][0] < (int(width)/2) and edge_points[1][0] < (int(width)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 
        
                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Q4
                    if edge_points[0][0] > (int(width)/2) and edge_points[1][0] > (int(width)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

                    #Between Q3 and Q4
                    if edge_points[0][0] < (int(width)/2) and edge_points[1][0] > (int(width)/2):
                        print("NOT Q3 or Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q3 or Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

                    if edge_points[0][0] > (int(width)/2) and edge_points[1][0] < (int(width)/2):
                        print("NOT Q3 or Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'NOT Q3 or Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (center_of_screen[0],center_of_screen[1]), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x2 = np.nan
                        y2 = np.nan

            #If the 2 coordinates in edge_points are at the corner of Q1
            if edge_points[0][1] == 0 and edge_points[1][0] == int(height):
                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q1
                    if edge_points[0][0] > (int(width)/2) and edge_points[1][1] < (int(height)/2):
                        print("Q1 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q1 = ' + '{:.2f}'.format(maxEllipse[2]), (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 + y_move

                        if x_check < 0 or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            if edge_points[0][0] == int(height) and edge_points[1][1] == 0:
                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q1
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][0] < (int(width)/2):
                        print("Q1 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q1 = ' + '{:.2f}'.format(maxEllipse[2]), (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 - x_move
                        y_check = y1 + y_move

                        if x_check < 0 or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            #If the 2 coordinates in edge_points are at the corner of Q2
            if edge_points[0][0] == 0 and edge_points[1][1] == 0:
                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q2
                    if edge_points[0][1] < (int(height)/2) and edge_points[1][0] < (int(width)/2):
                        print("Q2 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q2 = ' + '{:.2f}'.format(maxEllipse[2]), (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA)

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            if edge_points[1][0] == 0 and edge_points[0][1] == 0:
                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q2
                    if edge_points[1][1] < (int(height)/2) and edge_points[0][0] < (int(width)/2):
                        print("Q2 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q2 = ' + '{:.2f}'.format(maxEllipse[2]), (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA)

                        x_check = x1 + x_move
                        y_check = y1 + y_move

                        if x_check > width or y_check > height:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            #If the 2 coordinates in edge_points are at the corner of Q3
            if edge_points[0][0] == 0 and edge_points[1][1] == int(height):
                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q3
                    if edge_points[0][1] > (int(height)/2) and edge_points[1][0] < (int(width)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            if edge_points[0][1] == int(height) and edge_points[1][0] == 0:
                if int(maxEllipse[2]) > 0 and int(maxEllipse[2]) < 90:
                    #Q3
                    if edge_points[1][1] > (int(height)/2) and edge_points[0][0] < (int(width)/2):
                        print("Q3 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q3 = ' + '{:.2f}'.format(maxEllipse[2]), (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

                        x_check = x1 + x_move
                        y_check = y1 - y_move

                        if x_check > width or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            #If the 2 coordinates in edge_points are at the corner of Q4
            if edge_points[0][1] == int(height) and edge_points[1][0] == int(width):
                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q4
                    if edge_points[0][0] > (int(width)/2) and edge_points[1][1] > (int(height)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA)

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            if edge_points[0][0] == int(width) and edge_points[1][1] == int(height):
                if int(maxEllipse[2]) > 90 and int(maxEllipse[2]) < 180:
                    #Q4
                    if edge_points[0][1] > (int(height)/2) and edge_points[1][0] > (int(width)/2):
                        print("Q4 -> Angle = " + '{:.2f}'.format(maxEllipse[2]) + "\n")
                        cv.putText(img_capture, 'Q4 = ' + '{:.2f}'.format(maxEllipse[2]), (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA)

                        x_check = x1 - x_move
                        y_check = y1 - y_move

                        if x_check < 0 or y_check < 0:
                            x2 = np.nan
                            y2 = np.nan
                        else:
                            x2 = x_check
                            y2 = y_check

            if not (np.isnan(x2) or np.isnan(y2)): 
                #Draw arrow direction 
                img_capture = cv.arrowedLine(img_capture,(x1,y1),(x2,y2),(0,0,0),thickness=10)

        else:

            x1 = np.nan
            y1 = np.nan
            x2 = np.nan
            y2 = np.nan

        return (x1,y1),(x2,y2)
    
    def quadrant(self, img_capture, x, y):

        height,width = img_capture.shape

        # [0] = 1024, [1] = 1376
        #print(height) #[0]
        #print(width)  #[1]

        #print(int(img_wnd.size[1]),int(img_wnd.size[0]))
        center_of_screen = [int(width/2),int(height/2)]
        q1_x = int(center_of_screen[0] + (center_of_screen[0]/4))
        q1_y = int(center_of_screen[1] - (center_of_screen[1]/4))
        q2_x = int(center_of_screen[0] - (center_of_screen[0]/4))
        q2_y = int(center_of_screen[1] - (center_of_screen[1]/4))
        q3_x = int(center_of_screen[0] - (center_of_screen[0]/4))
        q3_y = int(center_of_screen[1] + (center_of_screen[1]/4))
        q4_x = int(center_of_screen[0] + (center_of_screen[0]/4))
        q4_y = int(center_of_screen[1] + (center_of_screen[1]/4))

        font = cv.FONT_HERSHEY_SIMPLEX 
        fontScale = 1
        colorText = (0,0,0)
        thickness = 2

        #Q1
        if x > (int(width)/2) and y < (int(height)/2):
            print("Q1 " + "\n")
            cv.putText(img_capture, 'Q1', (q1_x,q1_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

        #Q2
        if x < (int(width)/2) and y < (int(height)/2):
            print("Q2" + "\n")
            cv.putText(img_capture, 'Q2', (q2_x,q2_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

        #Q3
        if x < (int(width)/2) and y > (int(height)/2):
            print("Q3" + "\n")
            cv.putText(img_capture, 'Q3', (q3_x,q3_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

        #Q4
        if x > (int(width)/2) and y > (int(height)/2):
            print("Q4" + "\n")
            cv.putText(img_capture, 'Q4', (q4_x,q4_y), font, fontScale, colorText, thickness, cv.LINE_AA) 

        #Calculate delta of x,y point to center of screen
        delta_x_from_center = int(x-center_of_screen[0])
        delta_y_from_center = int(y-center_of_screen[1])

        return delta_x_from_center, delta_y_from_center