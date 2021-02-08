# NAME: Judah Zammit
# STID: 7839359
# UMNETID: zammitj3


# This file contains the intervels for several of the 
# colors present in the images
# As well as the colors for the various lines
from __future__ import absolute_import
import cv2
colors = {}

blue = {}
blue[u'low_H'] = 100
blue[u'low_S'] = 30 
blue[u'low_V'] = 0
blue[u'high_H'] = 120
blue[u'high_S'] = 255
blue[u'high_V'] = 255
colors[u'blue'] = blue

yellow = {}
yellow[u'low_H'] = 26
yellow[u'low_S'] = 31 
yellow[u'low_V'] = 218
yellow[u'high_H'] = 46
yellow[u'high_S'] = 255
yellow[u'high_V'] = 255
colors[u'yellow'] = yellow

red = {}
red[u'low_H'] = 167
red[u'low_S'] = 138
red[u'low_V'] = 0
red[u'high_H'] = 180
red[u'high_S'] = 255
red[u'high_V'] = 255
colors[u'red'] = red

green = {}
green[u'low_H'] = 27
green[u'low_S'] = 117
green[u'low_V'] = 0
green[u'high_H'] = 108
green[u'high_S'] = 255
green[u'high_V'] = 255
colors[u'green'] = green

white = {}
white[u'low_H'] = 0 
white[u'low_S'] = 0
white[u'low_V'] = 224
white[u'high_H'] = 173
white[u'high_S'] = 94
white[u'high_V'] = 255
colors[u'white'] = white

gray = {}
gray[u'low_H'] = 0 
gray[u'low_S'] = 0
gray[u'low_V'] = 153
gray[u'high_H'] = 180
gray[u'high_S'] = 89
gray[u'high_V'] = 189
colors[u'gray'] = gray
    

lineColors = {}
lineColors[u'unknown'] = (0,0,255)
lineColors[u'outer field'] = (0,255,0)
lineColors[u'inner field'] = (255,0,0)
lineColors[u'goal'] = (255,255,0)
lineColors[u'halfway'] = (255,0,255)
lineColors[u'middle circle'] = (0,255,255)
lineColors[u'obstacle'] = (90,180,255)
lineColors[u'goal rectangle'] = (180,255,90)
  
def drawLegend(frame):

    image = frame.copy()
    
    y = 10
    for key in lineColors.keys():
        # Draws the appropiate text inside of the goal
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,y)
        fontScale              = .5
        fontColor              = lineColors[key]
        lineType               = 2

        cv2.putText(image,key,
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)
        y += 15

    return image
