import matplotlib.image as mpimg
import cv2
import numpy as np
import os
def load_map(filename):
    import matplotlib.image as mpimg
    import cv2
    # print(os.getcwd())
    # im = cv2.imread("/home/manx52/catkin_ws/src/ROB521/labs/lab2/maps/myhal.png")
    # cv2.imshow('image', im)
    # im = cv2.flip(im, 0)
    # print(im)
    im = mpimg.imread("/home/manx52/catkin_ws/src/ROB521/labs/lab2/maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im)  #Whitespace is true, black is false
    # im_np = np.logical_not(im_np)     #for ros
    cv2.imshow('image', im)
    return im_np

map_filename = "willowgarageworld_05res.png"
occupancy_map = load_map(map_filename)



