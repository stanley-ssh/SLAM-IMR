# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3

from __future__ import division
from __future__ import absolute_import
import numpy as np
import cv2
import math
from seg import segment
from colors import colors,lineColors
from tools import *
from findObstacle import *
from findGoal import *

# Runs the fast line detector on an imag(or segmentation)
def FLD(image):

    # Create default Fast Line Detector class
    fld = cv2.ximgproc.createFastLineDetector(30,_canny_aperture_size = 3,_distance_threshold = 1.41421356, _do_merge = True)
    
    # Get line vectors from the image
    lines = fld.detect(image)
    
    # Draw lines on the image
    image = fld.drawSegments(image,lines)
    
    return lines,image

# Returns the slope and intercept of a line
def slope_intercept(line):

    # math stuff
    x1,y1,x2,y2 = line
    slope = (y2 - y1) / ((x2 - x1)+1e-10)# we add 1e-10 to avoid nan
    intercept = y1 - slope * x1
  
    return slope,intercept

# return the distance between two point
# (i.e. the lenght of a line
def lenght(line):
    return math.sqrt((line[0]-line[2])**2 + (line[1]-line[3])**2)


#  checks if two lines are similar
#  returns the merged line if they are similar
#  and none if they are not
#  l1: line 1
#  l2: line 2
#  parThresh: the threshold that we will use to determine if the lines are paralell
#  pointThresh: the threshold that we will use to determine if two points are close
#  drop: the x intercept of the line that determines the decay of the pointThresh hold
#         (see report for more details)
def jaggedSimilar(l1,l2, parThresh = 0,pointThresh = 0,drop = 0):

    # keeps track of whether the lines have been merged
    merged = False

    # holds the new line
    newStartPoint = None
    newEndPoint = None

    # Gets the slope and intercept of the rate of decay
    decaySI = slope_intercept((1000,1,drop,0))

    # Gets the decay line
    # will be 1 at 1000 and 0 at drop
    decayLine = lambda x:max(x*decaySI[0] + decaySI[1],0)
    
  
    # Ensures that l1 is always bigger then l2
    newL1 = None
    newL2 = None
    if lenght(l1) > lenght(l2):
        s1,i1 = slope_intercept(l1)    
        newL1 = l1
        s2,i2 = slope_intercept(l2)
        newL2 = l2
    else:
        s1,i1 = slope_intercept(l2)    
        newL1 = l2
        s2,i2 = slope_intercept(l1)
        newL2 = l1
    l1 = newL1
    l2 = newL2

    # Checks if the lines are paralell
    slopeSimilar = (s1 - parThresh) <= s2 <= (s1 +  parThresh)
    
    # If the slopes are not similar Do not merge,
    # From this point on, we can assume the lines ar paralell
    if not slopeSimilar:
        return None
    
    # Calculates the distacnce between the lines
    # (assuming they are paralall)
    distance = abs(i1 - i2) / np.sqrt((s1**2 + 1))
    
    # Checks if the smaller line is overlapping with the bigger one
    # (assuming they are paralall)
    overlap = (l2[0] + pointThresh) > l1[0] and l1[2] > ( l2[2] - pointThresh)

    # Checks if the end point of l1 and the start point
    # of l2 are close
    if (lenght((l1[0],l1[1],l2[2],l2[3])) 
        <= pointThresh*decayLine(l1[1])):
        
        # Merge with the start of l1 and the end point of l2
        merged = True 
        newStartPoint = (l2[0],l2[1])
        newEndPoint = (l1[2],l1[3])

    
    # Checks if the end point of l2 and the start point
    # of l1 are close
    if (lenght((l1[2],l1[3],l2[0],l2[1])) 
        <= pointThresh*decayLine(l2[1])):

        # Merge with the start of l1 and the end point of l2
        merged = True
        newStartPoint = (l1[0],l1[1])
        newEndPoint = (l2[2],l2[3])

    # Checks of the lines overlap and if the 
    # distance between them is close
    if (distance <= pointThresh*decayLine(l1[1])
         and overlap):
  
        # Merge by simply dropping the smaller line
        merged = True
        newStartPoint = (l1[0],l1[1])
        newEndPoint = (l1[2],l1[3])

        
    # If the lines have been merged, return the merged line
    # else return None
    if merged:
        return (int(newStartPoint[0]),int(newStartPoint[1]),
                int(newEndPoint[0]),int(newEndPoint[1]))
    else:
        return None


# merge an array of lines
#  lines: an array of lines
#  parThresh: the threshold that we will use to determine if the lines are paralell
#  pointThresh: the threshold that we will use to determine if two points are close
#  drop: the x intercept of the line that determines the decay of the pointThresh hold
#         (see report for more details)
def mergeLines(lines,parThresh=0,pointThresh=0,drop = 0):

    # This array holds merged lines
    newLines = []
  
    # Checks the len of both arrays
    oldLen = len(newLines)
    newLen = len(lines)

    # loop until there has been no change between iterations
    while not oldLen == newLen:
  
        # Check the lenght of the lines array before merging
        oldLen = len(lines)
    
        # Loop until lines is empyt
        while len(lines) > 0:
    
            # get the first line
            l1 = lines.pop(0)

            # Loop through the remaining lines
            i = 0
            while i < len(lines):

                # try merging lines[i] with l1
                newL1 = jaggedSimilar(l1,lines[i],parThresh,pointThresh,drop)

                # If succesful, remove it from lines[i] from lines
                # and replace l1 with the merged lines
                if not newL1 == None:
                    l1 = newL1
                    lines.pop(i)

                # move onto the next line
                i += 1
            
            # and the merged line to newLines
            newLines.append(l1)
  
        # reset lines and newLines
        lines = newLines
        newLines = []
        
        # check the lenght of lines after merging
        newLen = len(lines)        

    return lines

# Checks if a point is within a rectangle(within a threshold)
def within(point,rect,thresh = 0):

    x,y = point
    
    withinX = rect[0]-thresh <= x <= rect[0]+rect[2]+thresh
    withinY = rect[1]-thresh <= y <= rect[1]+rect[3]+thresh

    return withinX and withinY

# Ensures that all lines have a leftmost start point
def orderStartEnd(lines):

    newLines = []
    if lines == None:
	return newLines

    # order the start and the end based off of the x coordinate
    for line in lines:
  
        # If the line is already ordered just add it to newLines
        # else reorder the points
        line = line[0]
        if line[0] < line[2]:
            newLines.append(line)
        else:
            newLines.append((line[2],line[3],line[0],line[1]))    

    return newLines

# Remove any lines that are wihthin the obsticale rectangls
def removeObsLines(obs,lines,thresh = 20):

    newLines = []
    
    for line in lines:

        # Keeps track of whether the line must be removed
        remove = False
      
        # Check if the line is in each obs
        for ob in obs:

            # if the line is within this obstical or a previous one
            # we must remove it
            remove = remove or (within((line[0],line[1]),ob,thresh) 
                            and within((line[2],line[3]),ob,thresh))
            
        # If we don't have to remove the line, add it to newLines
        if not remove:
            newLines.append(line)
 
    return newLines


# Checks if a line is the Outer field line
# (Not the white lines but the borders of the field)
def checkOuterField(l,graySeg):

    # Finds the slope and intercept
    slope,inter = slope_intercept((l[u'x1'],l[u'y1'],l[u'x2'],l[u'y2']))
    
    # Pick the halfway point of the line
    halfX = int((l[u'x1'] + l[u'x2']) // 2)
    halfY = int(slope*halfX + inter)
    
    # Picks points in the four quadrants on either side of the point
    step = 10
    p1 = (halfX + step,halfY + step)
    p2 = (halfX - step,halfY + step)
    p3 = (halfX + step,halfY - step)
    p4 = (halfX - step,halfY - step)
  
    
    # If any of the points are gray, as determined by graySeg,
    # it is an outer field line
    outerField = False
    for p in [p1,p2,p3,p4]:
        
        # ensures that the points are within the image                
        x = max(p[0],0)
        x = min(x,len(graySeg[0])-1)
        y = max(p[1],0)
        y = min(y,len(graySeg)-1)
  
        # checks if that pixel is gray
        outerField = outerField or (graySeg[y,x] == 255)

    return outerField

# Finds all outer field lines
def findOuterField(lines,graySeg):
      
    # Changes the type of the line to 'outer field'
    # if it is an outerfield line
    for line in lines:
        if checkOuterField(line,graySeg):
            line[u'type'] = u'outer field'

    return lines    

# Find all inner field lines
def findInnerField(lines):

    # Seperates out the outer field lines
    outer = []
    rest  = []
    for l in lines:
        if l[u'type'] == u'outer field':
            outer.append(l)
        else:
            rest.append(l)

    # Checks if any of the non-outer field lines are inner field lines
    for l in rest:
  
        # gets the slope and intercept of l
        s1,i1 = slope_intercept((l[u'x1'],l[u'y1'],l[u'x2'],l[u'y2']))

        # Checks if l is parellal to any of the of the outer lines
        for out in outer:  
            
            # math stuff
            PAR_THRESH = .15
            s2,i2 = slope_intercept((out[u'x1'],out[u'y1'],out[u'x2'],out[u'y2']))
            slopeSimilar = (s1 - .15) <= s2 <= (s1 +  .15)

            # change type to inner field if the slopes are similar
            if slopeSimilar:
                l[u'type'] = u'inner field'
    
    # return all lines
    return outer + rest

# Find all Circle lines
def findCircleLines(lines):

    # Center line is not realible enough for this to work
    # I would find how close a line is to the center of the center
    # line and use that to determine if it is a circle line

    return lines

# Find all Goal lines
def findGoalLines(lines,goal):

    # Find bottom of the goal
    botStart = (goal[0],goal[1]+goal[3])
    botWH = (goal[2],0)

    # Get rectangle such that all lines within
    # this rectangle are goal lines
    THRESH = 150
    x = max(botStart[0] - THRESH,0)
    y = max(botStart[1] - THRESH,0)
    w = botWH[0] + THRESH*2
    h = THRESH*2
    rect = (x,y,w,h)

    # check if each line is within the rec
    for l in lines:
      s_within = within((l[u'x1'],l[u'y1']),rect) 
      e_within = within((l[u'x2'],l[u'y2']),rect) 
      if s_within and e_within:
          l[u'type'] = u'goal' 

    return lines

# Checks if l1 and l2 are perepindicular within a threshold
def perpindicular(l1,l2,thresh = 0):
  
    perp = False
    
    # get slopes of both lines
    s1,_ = slope_intercept((l1[u'x1'],l1[u'y1'],l1[u'x2'],l1[u'y2']))
    s2,_ = slope_intercept((l2[u'x1'],l2[u'y1'],l2[u'x2'],l2[u'y2']))
    
    # math stuff  
    perp = ((-1/(s2+1e-10))-thresh) <=  s1 <= ((-1/(s2+1e-10))+thresh)
 
    return perp

# Find all center lines
def findCenterLines(lines):

    # seperate out outer and inner lines
    outIn = []
    rest  = []
    for l in lines:
        if l[u'type'] == u'outer field' or l[u'type'] == u'inner field':
            outIn.append(l)
        else:
            rest.append(l)
    

    # Check if any lines are perpinduclar to the outer or inner lines
    for l in rest:
      
      for out in outIn:
        
        # if they are perpinducalar, change 'type' to ' center line
        PERP_THRESH =  3.9
        if perpindicular(l,out,PERP_THRESH):
            l[u'type'] = u'halfway' 

    # return all lines
    return rest + outIn

# Classifies the lines as the various line types
def identifyLines(lines,frame,goal = None):
    
    if goal == None:
        goal = findGoal(frame)

    newLines = []

    # converts the lines into dictionaries
    for line in lines:
        lineDict = {}
        lineDict[u'x1'] = line[0]
        lineDict[u'y1'] = line[1]
        lineDict[u'x2'] = line[2]
        lineDict[u'y2'] = line[3]
        lineDict[u'type'] = u'unknown'
        newLines.append(lineDict)
      
    # Finds all the outer field lines
    graySeg = segment(frame,colors[u'gray'])
    newLines = findOuterField(newLines,graySeg)
  
    # Finds all the inner field lines
    newLines = findInnerField(newLines)
    
    # Finds all the center Lines
    newLines = findCenterLines(newLines)

    # Find Goal Lines
    newLines = findGoalLines(newLines,goal)
    
    # Find Circle Lines
    newLines = findCircleLines(newLines)

    return newLines

# Find all the lines
def findLines(frame,obs = None,goal = None):

    # If the caller did not give us a obs or goal, get our own
    if obs == None:
        obs = findObstacle(frame)
    if goal == None:
        goal = findGoal(frame)

    # Segment the green field
    seg = segment(frame,colors[u'green'],True)

    # Get the lines from FLD 
    lines,seg = FLD(seg)
    
    # Order the lines based of off the x coord
    lines = orderStartEnd(lines)
    
    # Remove any lines within the obstacles
    lines = removeObsLines(obs,lines)

    # Merge all the lines 
    PAR_THRESH = .1
    POINT_THRESH = 135
    DROP = 250

    lines = mergeLines(lines,PAR_THRESH,POINT_THRESH,DROP)

    # identify the lines in the image
    lines = identifyLines(lines,frame,goal)
      
    return lines

# draws all the lines
def drawLines(image,lines):

    # ...draws the lines
    for line in lines:
        cv2.line(image, (line[u'x1'], line[u'y1']), (line[u'x2'], 
                  line[u'y2']), lineColors[line[u'type']], 2)

    return image
