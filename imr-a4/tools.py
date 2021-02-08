# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

import numpy as np
import cv2

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

# reads a certain image
def read(index):
    return cv2.imread('./img/{}.jpg'.format(index))


# reads the first nine images and stacks them
def get_test():
  
    lis = [[],[],[]]
    
    # read first images
    for i in range(3):
        for j in range(3):
            lis[i].append(read((i+1)*(j+1) - 1))

    new_lis = []
    for x in lis:
        new_lis.append(np.hstack(x))

    frame = np.vstack(new_lis)  

    frame = downscale(frame,.3)

    return frame
