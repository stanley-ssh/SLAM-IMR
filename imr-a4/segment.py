# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

# THis file was used to find good segmentation thresholds
# It does not get used in the final program

from __future__ import division
from __future__ import absolute_import
import numpy as np
import cv2
from seg import segment
SCALE = .5

max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = u'Video Capture'
window_detection_name = u'Object Detection'
low_H_name = u'Low H'
low_S_name = u'Low S'
low_V_name = u'Low V'
high_H_name = u'High H'
high_S_name = u'High S'
high_V_name = u'High V'
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)
cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)


# Down sizes the image by a certain percent
def downscale(img,percent):
    
    # Finds the new height and width
    width = int(img.shape[1] * percent)
    height = int(img.shape[0] * percent)
    dim = (width, height)

    # resize image
    resized = cv2.resize(img, dim) 
    
    return resized

# Up sizes the image by a certain percent
def upscale(img,percent):
    
    # Finds the new height and width
    width = int(img.shape[1] / percent)
    height = int(img.shape[0] / percent)
    dim = (width, height)

    # resize image
    resized = cv2.resize(img, dim) 
    
    return resized

def read(index):
    return cv2.imread(u'./img/{}.jpg'.format(index))


def get_test():
    lis = [[],[],[]]
    # read first image
    for i in xrange(3):
        for j in xrange(3):
            lis[i].append(read((i+1)*(j+1)))

    new_lis = []
    for x in lis:
        new_lis.append(np.hstack(x))

    frame = np.vstack(new_lis)  
    frame = downscale(frame,.3)
    return frame


frame = read(1)
# Display the output



while True:
    blue = {}
    blue[u'low_H'] = low_H
    blue[u'low_S'] = low_S
    blue[u'low_V'] = low_V
    blue[u'high_H'] = high_H
    blue[u'high_S'] = high_S
    blue[u'high_V'] = high_V

    seg = segment(frame,blue)
    cv2.imshow(u'hsv',frame) 
    cv2.imshow(u'org',seg) 
    

    key = cv2.waitKey(30)
    if key == 27:#if ESC is pressed, exit loop
        cv2.destroyAllWindows()
        break
