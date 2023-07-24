import time
import os
import numpy as np
		
def main():
        tip_coord_list = np.array([ [ 473., 760.], [ 419., 954.], [ 415., 990.], [ 509., 681.],
                                    [ 508., 680.], [ 326.,1172.], [ 509., 680.], [ 485., 648.],
                                    [ 508., 680.], [ 357.,1162.], [ 509., 674.], [ 508., 680.],
                                    [ 509., 681.], [ 400.,1081.], [ 509., 681.], [ 509., 681.],
                                    [ 509., 681.], [ 508., 680.]])
        print(tip_coord_list)

		#Histogram of all the x,y coordinate pixels for tip

        reson = 10

        centre_coord = np.array([688,512])

        tip_coord_list_x = tip_coord_list[:,1]
        tip_coord_list_y = tip_coord_list[:,0]


        tip_coord_list_x_range = np.max(tip_coord_list_x) - np.min(tip_coord_list_x)
        tip_coord_x_hist, tip_coord_x_edge = np.histogram(tip_coord_list_x,int(tip_coord_list_x_range/reson))
        bin_idx_x = np.argmax(tip_coord_x_hist)
        
        hist, edge = np.histogram(tip_coord_list_x,np.arange(np.min(tip_coord_list_x),np.max(tip_coord_list_x),1))
        print(f"hist: {hist}")
        print(f"edge: {edge}")
        print(f"hist.size: {hist.size}")
        moving_avg = np.empty(shape = (hist.size - reson))
        for idx in range(hist.size - reson):
            moving_avg[idx] = np.sum(hist[idx : idx + reson])
        print(f"moving avg: {moving_avg}")
        moving_avg_idx = np.argwhere(moving_avg == np.amax(moving_avg))
        print(f"moving avg idx: {moving_avg_idx}")
        print(f"moving avg idx.size: {moving_avg_idx.size}")
        for idx in range(moving_avg_idx.size):
            print(moving_avg[moving_avg_idx[idx]])
            
        moving_avg_idx_diff = np.diff(moving_avg_idx, axis = 0)
        print(moving_avg_idx_diff)
        print(np.ones(shape = [moving_avg_idx_diff.size, 1]))
        if moving_avg_idx_diff == np.ones(shape = moving_avg_idx_diff.size):
            print(moving_avg_idx_diff)

        tip_coord_list_y_range = np.max(tip_coord_list_y) - np.min(tip_coord_list_y)
        tip_coord_y_hist, tip_coord_y_edge = np.histogram(tip_coord_list_y,int(tip_coord_list_y_range/reson))
        bin_idx_y = np.argmax(tip_coord_y_hist)




        tip_coord_most = np.array([int(np.average([tip_coord_x_edge[bin_idx_x], tip_coord_x_edge[bin_idx_x + 1]])), int(np.average([tip_coord_y_edge[bin_idx_y], tip_coord_y_edge[bin_idx_y + 1]]))])

        print(f"centre distance = {np.linalg.norm(centre_coord - tip_coord_most)}")

        print(f"tip coord most = {tip_coord_most}")
        

if __name__ == '__main__':
    main()