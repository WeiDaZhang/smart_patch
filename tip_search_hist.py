import time
import os
import numpy as np
import matplotlib.pyplot as plt

def plot_surf(coord_list):
    coord_list = np.array(coord_list)
    map_range = [ np.max(coord_list[:, 0]), np.max(coord_list[:, 1])]

    print(f"Map Range: {map_range}")
    z = np.zeros((int(map_range[0] + 1),int(map_range[1] + 1)))
    for idx in range(np.shape(coord_list)[0]):
        z[int(coord_list[idx, 0]), int(coord_list[idx, 1])] = z[int(coord_list[idx, 0]), int(coord_list[idx, 1])] + 1
    plt.style.use('_mpl-gallery-nogrid')

    # plot
    fig, ax = plt.subplots()

    ax.imshow(z)

    plt.show()

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
    print(f"hist: {coord_hist}")
    print(f"edge x: {coord_edge_x}")
    print(f"edge y: {coord_edge_y}")
        
    if np.min([coord_range[0] + 1, coord_range[1] + 1]) < resolution:
        resolution = np.min(coord_range)
        print(f'Resolution changed to: {resolution}')

    search_range = [int(coord_range[0] + 1 - resolution + 1),
                    int(coord_range[1] + 1 - resolution + 1)]
    print(f"Search Range: {search_range}")
    
    hist_cum = np.empty(search_range)
    for idy in range(0, search_range[1]):
        for idx in range(0, search_range[0]):
            hist_cum[idx, idy] = np.sum(coord_hist[idx : idx + resolution, idy : idy + resolution])
            
    cum_idn = np.unravel_index(hist_cum.argmax(), hist_cum.shape)
    return [coord_org[0] + cum_idn[0], coord_org[1] + cum_idn[1]]
        

if __name__ == '__main__':
    all_coord_lists = read_coord()
    for coord_list in all_coord_lists:
        plot_surf(coord_list)
        print(hist_top_coord(coord_list, 3))