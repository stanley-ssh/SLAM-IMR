from CONSTANTS import *
from findLines import slope_intercept
 
def EOFInFrontHelper(frame,line,local = False):

	# get points
	x1 = line['x1']
	y1 = line['y1']
	x2 = line['x2']
	y2 = line['y2']
		
	# subtract half of the x axis so that
	# the y axis is in the middle
	x1 -= frame.shape[1]//2
	x2 -= frame.shape[1]//2

	# get the intercept
	_,intercept = slope_intercept((x1,y1,x2,y2))	

	# is the intercept within the lower half of the image
	if local:
		imm = (frame.shape[0] > intercept > (frame.shape[0]-frame.shape[0]//SOFT_EOF_THRESH))
	else:
		imm = (frame.shape[0] > intercept > (frame.shape[0]-frame.shape[0]//EOF_THRESH))
	# Is the intercept actually on the line
	on = (x1*x2)<0
	
	return imm and on	

def EOFInFront(frame,lines,local = False):

	immenent_line = None
	for line in lines:
		if line['type'] == 'outer field': 
			if EOFInFrontHelper(frame,line,local):
				immenent_line = line
				break
	

	return immenent_line


# Tests if the goal is in front of the robot
# that is, if we move the robot forward will
# it enter the goal
def goalInFront(frame,goal,exact = False):
	
	# Find the center of the image
	# ie the trajectory of the robot
	# It is enough to use x because
	# the robot canot change it's height
	xLen = frame.shape[1]
	center = xLen//2

	goalCenter = goal[0] + (goal[2]//2)

	thresh = GOAL_IN_FRONT_THRESH*goal[2]

	# Checks if the goal center is in the center of the image
    	withinX = goalCenter-thresh  <= center <= goalCenter+thresh 
	
	# if we want the exact position
	if exact:
		# if we are looking at the goal
		if withinX:
			# return the percentege that we are in the goal
			return float(center - goal[0])/float(goal[2])
		else:
			return -1
	else:
		return withinX

# Checks if the robot is obstructed
def obstruction(frame,obs):
	
	inTheWay = None
	
	for ob in obs:

		# Find the center of the image
		# ie the trajectory of the robot
		# It is enough to use x because
		# the robot canot change it's height
		xLen = frame.shape[1]
		center = xLen//2

	
		thresh = float(ob[3])/OBSTRUCTION_IN_THE_WAY_THRESH 
		withinX = ob[0]-thresh <= center <= ob[0]+ob[2]+thresh

		if withinX:
			if inTheWay == None:
				inTheWay = [ob]
			else:
				prop = float(ob[3])/float(inTheWay[0][3])
				within = OBSTRUCTION_SAME_SIZE_THRESH
				if prop < within:
					inTheWay[0] = inTheWay[0]
				elif (1/prop) < within:
					inTheWay[0] = ob
				else:
					inTheWay.append(ob)
						

	return inTheWay
	
# Checks if the goal is close
def goalHo(frame,goal,thresh):

	if goal == None:
		return False	

	# Get the proportion of the frame that is filled with the goal
	frameLen = frame.shape[1]
	frameLen = float(frameLen)
	goalLen = goal[2]
	goalLen = float(goalLen)
	prop = goalLen/frameLen
	
	# Checks that the goal is close
	goalClose = prop >= thresh

	return goalClose

# Checks if the obs is close
def obsHo(frame,obs,thresh):

	if obs == None:
		return False	
	else:
		# If there are many obstacles in the way
		# just pick one as they will all be roughly the same size
		obs = obs[0]

	# Get the proportion of the frame that is filled with the obstacle
	frameLen = frame.shape[0]
	frameLen = float(frameLen)
	obsLen = obs[3]
	obsLen = float(obsLen)
	prop = obsLen/frameLen
	
	# Checks that the goal is close
	obsClose = prop >= thresh
	
	return obsClose


# Decide what direction to turn to get to the goal
def goalDir(frame,goal):
	
	# Find the Image center
	xLen = frame.shape[1]
	center = xLen//2

	# Find the goal Center
	goalCenter = goal[0] + (goal[2]//2)

	# Check the offset from the center to the goal center
	diff = center - goalCenter

	# Return the direction of the goal
	if diff >= 0:
		return 'left'
	else:
		return 'right'

def obsDir(frame,obs):

	if obs == None:
		# this should not get used
		return None	
	else:
		# If there are many obstacles in the way
		# just pick one as they will all be roughly the same size
		obs = obs[0]

	# find the image center
	xLen = frame.shape[1]
	center = xLen//2

	# find the obstacle center
	obsCenter = obs[0] + (obs[2]//2)

	# Check the offset from the center to the obs center
	diff = center - obsCenter

	# Return the direction away from the obstacle
	if diff >= 0:
		return 'right'
	else:
		return 'left'

# Checks if the goal is in view
def goalInSight(goal,frame):
	
	# Checks if it within one quarter of the left edge
	close = goal[0] >= frame.shape[1]/GOAL_IN_SIGHT_THRESH
	
	# Checks if it within one quarter of the right edge
	far = goal[0]+goal[2] <= (GOAL_IN_SIGHT_THRESH-1)*frame.shape[1]/GOAL_IN_SIGHT_THRESH
	
	return far and close
