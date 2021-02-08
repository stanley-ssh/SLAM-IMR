# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

from __future__ import absolute_import
from seg import segment
import numpy as np
import cv2
from colors import colors
from CONSTANTS import *


# Checks if two 1d lines overlap within a thresh hold
def similar_helper(x,x1,thresh = 0):

    (x,w),(x1,w1) = x,x1

    # Find the rect angle with the smallest width
    if w < w1:
      x_s,w_s = (x,w)
      x_b,w_b = (x1,w1)
    else:
      x_s,w_s = (x1,w1)
      x_b,w_b = (x,w)

    # Adds the threshold to either end of the bigger line
    x_b = x_b - thresh
    w_b = w_b + thresh*2
  
    # Checks if either of the start points of the smaller line
    # are within the bigger line
    x_overlap = (x_s >= x_b) and (x_s <= (x_b + w_b))
    x_overlap = (((x_s + w_s) >= x_b) and ((x_s + w_s) <= (x_b + w_b))) or x_overlap
 
    return x_overlap

# Checks if two rectangles are overlapping within a certain threshold
def similar(rect1,rect2,thresh = 0):

    # Checks if both the x lines and y lines overlap
    x_overlap = similar_helper((rect1[0],rect1[2]),(rect2[0],rect2[2]),thresh)
    y_overlap = similar_helper((rect1[1],rect1[3]),(rect2[1],rect2[3]),thresh)

    return x_overlap and y_overlap

# Merges two rectangles together
def rect_merge(rect1,rect2):

    # Finds the extreme points of both rectangles and 
    # draws a rectangle using them
    x = min(rect1[0],rect2[0])
    y = min(rect1[1],rect2[1])
    w = max(rect1[0]+rect1[2],rect2[0]+rect2[2]) - x
    h = max(rect1[1]+rect1[3],rect2[1]+rect2[3]) - y

    return x,y,w,h


# Merges all the rectangles into rectangles that can no longer be merged
def merge_all(obs_master,thresh = 0):

    # obs is all the un merged rectangles
    obs = obs_master[:]

    # new obs is all the merged rectangles
    new_obs = []
  
    new_len = len(new_obs)
    old_len = len(obs) 

    # if the lenght has not changed within iterations, stop merging
    while not new_len == old_len:

        # check the lenght of the list at the start
        old_len = len(obs)

        # Loop until no more unmerged rectangles
        while len(obs) > 0:

            to_delete = []
            
            # get the top rectangle
            curr = obs.pop()
            
            # Loop through the remaining rectangles and 
            # merge any similar rectangles
            for i in xrange(len(obs)):
              
                # if similar, merge into curr
                if similar(curr,obs[i],thresh):
                    curr = rect_merge(curr,obs[i])
                    to_delete.append(obs[i])
            
            # delete all the rectangles that have been merged into curr
            for rect in to_delete:
                obs.remove(rect)
  
            # move curr into new obs
            new_obs.append(curr)
            
        # Check the new lenght
        new_len = len(new_obs)
        obs = new_obs[:]

        # reset
        new_obs = []

    new_obs = []
    while len(obs) > 0:
	max_pos = 0
	for i in range(len(obs)-1):
		if obs[i][3] > obs[max_pos][3]:
			max_pos = i
	new_obs.append(obs.pop(max_pos))

    return new_obs

# Gets the rectangles from the segmentation
def segToRect(seg):

        # Get contours
        rect,contours,hierarchy = cv2.findContours(seg, 1, 2)
        
        # Convert countours to rectangles
        out = []
        for cnt in contours:
          out.append(cv2.boundingRect(cnt))       
       
        return out

# Checks if obsticles are of similar size
def sameSize(ob1,ob2):
	prop = float(ob1[3])/float(ob2[3])
	if prop > 1:
		prop = 1/prop
	return prop > .5


# seperate the obs into partitions based on area
def partitionObs(obs):
	partObs = []
	while len(obs) != 0:
		# get the first obs and find all others that are of similar size
		curr = obs.pop()
		similar = []
		for ob in obs:
			if sameSize(curr,ob):
				similar.append(ob)

		for ob in similar:
			obs.remove(ob)
		similar.append(curr)
		partObs.append(similar)
				
	return partObs

def filterObs(obs):
	newObs = []
	for x in obs:
		if x[3] > RED_BOX_IGNORE_THRESH:
			newObs.append(x)

	return newObs

def removeDouble(obs):

	new_obs = []
	
	while len(obs)>0:
		curr = obs.pop()
		same = None
		for i in range(len(obs)):
			ob = obs[i]
			thresh = 40
			same_size = (ob[3] + thresh > curr[3] > ob[3] - thresh)
			same_x = (ob[0] + thresh > curr[0] > ob[0] - thresh)
			if same_x:
				same = i
		if same == None:
			new_obs.append(curr)
		else:
			obs.pop(same)
	return new_obs

# Returns a list of obstacles
def findObstacle(frame,merge = True,removeSmall = True):
    
  # Segments the red checkers
  seg = segment(frame,colors[u'red'])
  kernel = np.ones((5,5),np.uint8)
  seg = cv2.erode(seg,kernel,iterations = 1)

  # Get the small rectangels
  obs = segToRect(seg)
  if removeSmall:
  	obs = filterObs(obs)
	
  return removeDouble(merge_all(obs,RED_BOX_MERGE_THRESH))

# Draws the obstacles on the image
def drawObstacle(frame,ob): 

  # make a copy of the image
  image = frame.copy()

  # Loop through all the obstacles
  for obs in ob:
    
    x,y,w,h = obs
    
    # Finds the center and the area of the goal
    center = (x+(w//2),y+(h//2))
    area = int(w*h)
    
    # Draw the rectangles
    cv2.rectangle(image,(x,y),(x+w,y+h),(90,180,255),2)
  
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
