# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

import numpy as np
import cv2

# Segments a certain color in the image
def segment(frame,color,flip =  False):
      
        # convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # threshold the image
        seg = cv2.inRange(hsv, (color['low_H'], color['low_S'], color['low_V']), 
                                (color['high_H'], color['high_S'], color['high_V']))
        
        # if flip is set, turn white to black and black to white
        if flip:
            seg = seg -255
            seg = seg*-1
            seg = seg.astype('uint8')

        # Do two opening operations
        kernel= np.ones((2,2), np.uint8)
        seg = cv2.morphologyEx(seg, cv2.MORPH_OPEN, kernel,iterations = 1)
        kernel= np.ones((2,2), np.uint8)
        seg = cv2.morphologyEx(seg, cv2.MORPH_OPEN, kernel,iterations = 1)
        
        return seg
