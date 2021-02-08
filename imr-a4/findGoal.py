# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

from __future__ import absolute_import
from seg import segment
import numpy as np
import cv2
from colors import colors

# This function takes a binary segmentation finds the rectangle asscociated with the goal
def segToRect(seg):
      
        # Here we cheat and say that if there is no goal
        # the goal is one pixel in the corner of the image
        x = 0  
        y = 0  
        w = 0  
        h = 0  

        # Finds the contors
        rect,contours,hierarchy = cv2.findContours(seg, 1, 2)
        
        # This is for the case where the whole goal in view
        if len(contours) == 1:
                
                # Get's the first and only contor
                cnt = contours[0]
    
                # Finds the rectangle that contains the rectangle
                bbox = cv2.boundingRect(cnt)
                x,y,w,h = bbox
    
        # This is for the case where only the two pillars are present in the image
        elif len(contours) == 2:
              
                # Pillar 1  
                cnt = contours[0]
                bbox = cv2.boundingRect(cnt)
                x_1,y_1,w_1,h_1 = bbox
                
                
                # Pillar 2
                cnt = contours[1]
                bbox = cv2.boundingRect(cnt)
                x_2,y_2,w_2,h_2 = bbox

                # Merge the two rectangles into one
                x = min(x_1,x_2)
                y = min(y_1,y_2)
                w = max(x_1+w_1,x_2+w_2) - x
                h = max(y_1+h_1,y_2+h_2) - y
               
        return x,y,w,h

# Finds the goal in the image
def findGoal(frame,other = False):
    
  # Segments the blue in the image
  if other:
  	seg = segment(frame,colors[u'yellow'])
  else:
  	seg = segment(frame,colors[u'blue'])
	
  kernel = np.ones((5,5),np.uint8)
  seg = cv2.erode(seg,kernel,iterations = 1)
  
  # Gets the box from the coordinates
  x,y,w,h = segToRect(seg)

  return (x,y,w,h)


# draw the goal on the image
def drawGoal(frame,goal): 

  # Gets the x,y,w,h coordinates of the goal
  x,y,w,h = goal
  
  # Finds the center and the area of the goal
  center = (x+(w//2),y+(h//2))
  area = int(w*h)
  
  # Make a copy and draw the goal on that copy
  image = frame.copy()
  cv2.rectangle(image,(x,y),(x+w,y+h),(180,255,90),2)


  # Draws the appropiate text inside of the goal
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  bottomLeftCornerOfText = center
  fontScale              = .5
  fontColor              = (0,0,0)
  lineType               = 2

  cv2.putText(image,u'({},{},{})'.format(center[0],center[1],area), 
      bottomLeftCornerOfText, 
      font, 
      fontScale,
      fontColor,
      lineType)

  return image
