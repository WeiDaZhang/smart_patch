import time
import os
import numpy as np
import matplotlib.pyplot as plt

def generate_circle_pixels(centre_coord, radius):
    circumference = np.pi * 2 * radius
    circumf_pixels = (np.rint(circumference)).astype(int)
    theta_list = 2 * np.pi * np.arange(0, circumf_pixels) / circumf_pixels
    circle_pixels = np.array([np.sin(theta_list) * radius, np.cos(theta_list) * radius])
    centre_coord.shape = (1, 2)
    return centre_coord + np.transpose(circle_pixels)

def add_points_2_img(coord_list, img = np.zeros([0, 0])):
    coord_list = np.array(coord_list)
    map_range = (np.rint([ np.max(coord_list[:, 0]), np.max(coord_list[:, 1])])).astype(int)

    print(f"Adding Points Coordinates Range to Display: {map_range}")
    if img.shape[0] == 0 and img.shape[1] == 0:
        img = np.zeros([int(map_range[0] + 1), int(map_range[1] + 1)])
    elif img.shape[0] < map_range[0] and img.shape[1] < map_range[1]:
        img = np.append(img, np.zeros([img.shape[0], map_range[1] - img.shape[1] + 1]), axis = 1)
        img = np.append(img, np.zeros([map_range[0] - img.shape[0] + 1, map_range[1]]), axis = 0)
    elif img.shape[0] < map_range[0]:
        img = np.append(img, np.zeros([map_range[0] - img.shape[0] + 1, img.shape[1]]), axis = 0)
    elif img.shape[1] < map_range[1]:
        img = np.append(img, np.zeros([img.shape[0], map_range[1] - img.shape[1] + 1]), axis = 1)

    for idx in range(np.shape(coord_list)[0]):
        img[int(coord_list[idx, 0]), int(coord_list[idx, 1])] = img[int(coord_list[idx, 0]), int(coord_list[idx, 1])] + 1

    return img

def plot_image(img, fig = []):
    plt.style.use('_mpl-gallery-nogrid')
    
    # plot
    if not fig:
        fig, ax = plt.subplots()
    else:
        ax = plt.gca()

    ax.imshow(img)

    plt.ion()
    plt.show(block = False)
    plt.pause(0.01)
    
    return fig

def read_coord():
    f = open("histogram coordinates.txt", "r")
    coord_list = []
    all_coord_lists = []
    text_lines = f.readlines()
    for text in text_lines:
        text = text.strip()
        if not text:
            coord_list.clear()
            continue
        elif text[0] == '[':
            if text[1] == '[':
                char_start = 2
            else:
                char_start = 1
            if text[-2] == ']':
                char_stop = -2
            else:
                char_stop = -1
            text_list = text[char_start:char_stop].split()
            coord_list.append([float(text_list[0]), float(text_list[1])])
            if char_stop == -2:
                all_coord_lists.append(coord_list.copy())

    return all_coord_lists

def search_centralized_coordinate(coord_list, radius):
    #   coord_list: numpy array with shape of (x, 2)
    #   radius:     centric radius
    centre_coord = np.mean(coord_list, axis = 0)
    centre_coord_distance = np.linalg.norm(coord_list - centre_coord, axis = 1)
    centre_coord_mean_distance = np.mean(centre_coord_distance)
    print(f'Coordinate List Centre Distance: {centre_coord_mean_distance}')
    
    dstr_coord_list = np.empty([0,2])
    while centre_coord_mean_distance > radius * np.sqrt(2):
        kick_coord = coord_list[np.argmax(centre_coord_distance),:].copy()
        kick_coord.shape = (1, 2)
        dstr_coord_list = np.append(dstr_coord_list, kick_coord, axis = 0)
        coord_list = np.delete(coord_list, np.argmax(centre_coord_distance), 0)
        
        centre_coord = np.mean(coord_list, axis = 0)
        centre_coord_distance = np.linalg.norm(coord_list - centre_coord, axis = 1)
        centre_coord_mean_distance = np.mean(centre_coord_distance)
        print(f'Updated Coordinate List Centre Distance: {centre_coord_mean_distance}')
    print(f'Centralized Coordinate: {centre_coord}')
    print(f'Centralized Coordinate Occurrence: {coord_list.shape[0]}')
    print(f'Distributed Coordinates: {dstr_coord_list}')
    return centre_coord, coord_list.shape[0], dstr_coord_list

def hist_top_coord(coord_list, resolution = 10):
    coord_list = np.array(coord_list)
    print(f"Coordinate List: {coord_list}")
    

    coord_org = [np.min(coord_list[:, 0]), np.min(coord_list[:, 1])]

    coord_range = [ np.max(coord_list[:, 0]) - np.min(coord_list[:, 0]),
                    np.max(coord_list[:, 1]) - np.min(coord_list[:, 1])]

    print(f"Coordinate Range: {coord_range}")
    
    coord_hist, coord_edge_x, coord_edge_y = np.histogram2d(coord_list[:, 0],
                                                            coord_list[:, 1],
                                                            (np.ceil(coord_range)).astype(int))
    #print(f"hist: {coord_hist}")
    #print(f"edge x: {coord_edge_x}")
    #print(f"edge y: {coord_edge_y}")
        
    if np.min([coord_range[0] + 1, coord_range[1] + 1]) < resolution:
        resolution = np.min(coord_range)
        print(f'Resolution changed to: {resolution}')

    search_range = (np.rint([coord_range[0] + 1 - resolution + 1,
                            coord_range[1] + 1 - resolution + 1])).astype(int)
    print(f"Search Range: {search_range}")
    
    hist_cum = np.empty(search_range)
    for idy in range(0, search_range[1]):
        for idx in range(0, search_range[0]):
            hist_cum[idx, idy] = np.sum(coord_hist[ idx : (np.rint(idx + resolution)).astype(int),
                                                    idy : (np.rint(idy + resolution)).astype(int)])
            
    cum_idn = np.unravel_index(hist_cum.argmax(), hist_cum.shape)
    
    all_cum_idn = np.argwhere(hist_cum == hist_cum[cum_idn]) + np.array([resolution/2, resolution/2])
    print(f'Top Histogram Index List: {all_cum_idn}')
    all_cum_idn_cnt =  all_cum_idn.shape[0]
    
    centre_idn_occur_list = np.array([])
    centre_idn_list = np.empty([0,2])
    if all_cum_idn_cnt > 1:
        while not all_cum_idn.size == 0:
            centre_idn, centre_idn_occur, all_cum_idn = search_centralized_coordinate(all_cum_idn, resolution)
            centre_idn_occur_list = np.append(centre_idn_occur_list, centre_idn_occur)
            centre_idn.shape = (1, 2)
            centre_idn_list = np.append(centre_idn_list, centre_idn, axis = 0)
    else:
        centre_idn_list = all_cum_idn
        centre_idn_occur_list = np.append(centre_idn_occur_list, 1)
    top_coord_list = coord_org + centre_idn_list
    top_coord_chance = centre_idn_occur_list/all_cum_idn_cnt
    print(f'Top Histogram Coordinate List: {top_coord_list}')
    return top_coord_list[0,:], top_coord_chance[0], top_coord_list[1:-1,:], top_coord_chance[1:-1]
        

def get_tip_coord(coord_list, coord_avg):
    if not np.isnan(coord_avg):
        minx_idx = np.argmin(coord_list[:, 0])
        minx_idx_coord = coord_list[minx_idx]
        tip_coord = np.array([[int(minx_idx_coord[1]), int(minx_idx_coord[0])]])

    else:
        tip_coord = coord_avg

    return tip_coord

if __name__ == '__main__':
    all_coord_lists = read_coord()
    for coord_list in all_coord_lists:
        img = add_points_2_img(coord_list)
        fig = plot_image(img)
        top_coord, top_coord_chance, rst_coord_list, rst_coord_chance = hist_top_coord(coord_list, 4)
        img = add_points_2_img(generate_circle_pixels(top_coord, 10), img)
        plot_image(img, fig)
    input('Enter to Exit and Close All Images')