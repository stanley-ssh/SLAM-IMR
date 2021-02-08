import threading
from scipy.stats import norm
gaussian_prob = norm.pdf
import rospy
from geometry_msgs.msg import Twist
from math import radians
import cv2
from robot import Robot
import numpy as np
import math
from seg import segment
from colors import colors,lineColors,drawLegend
from tools import *
from findLines import *
from findObstacle import *
from findGoal import *
import sys
from imageProcess import *
from CONSTANTS import *


# FILE LEVEL VARIABLES

# Prep Robot
robot=Robot()
rate=rospy.Rate(RATE)
twist=Twist()

# Allow the image to be shown at all times
currIm = np.zeros((512,512))
nextIm = np.zeros((512,512))

# soccer map
soc_map = cv2.imread('./soccer_field.png')
#cv2.rectangle(soc_map,(175-25,125-25),(175+25,125+25)
#		,(0,0,255),-1)
#cv2.rectangle(soc_map,(215-25,480-25),(215+25,480+25)
#		,(0,0,255),-1)
#cv2.rectangle(soc_map,(75-25,300-25),(75+25,300+25)
		#,(0,0,255),-1)

# Initialize all
imFeatures = {}
imFeatures['Goal Ahead'] = False
imFeatures['Other Goal Ahead'] = False
imFeatures['Goal Pos'] = -1
imFeatures['Other Goal Pos'] = -1
imFeatures['Obstacles'] = None
imFeatures['Big Goal'] = False
imFeatures['Obs Direction'] = 'left' 	
imFeatures['Big Obs'] = False
imFeatures['Times Looking For Goal'] = DEADLOCK_BREAK
imFeatures['Goal Direction'] = 'left'
imFeatures['Other Goal Direction'] = 'left'
imFeatures['Obstructed'] = False
imFeatures['Goal In Sight'] = False
imFeatures['Other Goal In Sight'] = False
imFeatures['EOF Line Ahead'] = False
imFeatures['Soft EOF Line Ahead'] = False
imFeatures['Homerun'] = False
imFeatures['Goal Occluded'] = True
imFeatures['Other Goal Occluded'] = True
imFeatures['Goal Dist'] = 'none'
imFeatures['Other Goal Dist'] = 'none'
imFeatures['All Obstacles'] = None

# Robot Speed
angularZ = radians(0)
linearX = 0

# All the particles are at the initial position
if INITIAL_NOTIFICATION:
	x = np.ones((PARTICLES))*soc_map.shape[1]//2
	y = np.ones((PARTICLES))*soc_map.shape[0]
	theta = np.ones((PARTICLES))*270 
	weights = np.ones((PARTICLES))
	loc = np.stack([y,x,theta,weights],axis = -1)
	obs_mean = np.zeros((PARTICLES,MAX_OBS,2)) - 1
	obs_var = np.zeros((PARTICLES,MAX_OBS,2))
	obs_num = np.zeros(PARTICLES)
	obs_used = np.ones((PARTICLES,MAX_OBS))
else:
	x = np.random.randint(0,300,PARTICLES)
	y = np.random.randint(0,600,PARTICLES)
	theta = np.random.randint(0,360,PARTICLES)
	weights = np.ones((PARTICLES))
	loc = np.stack([y,x,theta,weights],axis = -1)
	obs_mean = np.zeros((PARTICLES,MAX_OBS,2)) - 1
	obs_var = np.zeros((PARTICLES,MAX_OBS,2))
	obs_num = np.zeros(PARTICLES)
	obs_used = np.ones((PARTICLES,MAX_OBS))


def drawRobotTriangle(this_map,robot_x,robot_y,robot_theta):
		
	# hella trig
	rad = np.radians(robot_theta)
	x = robot_x + np.cos(rad)*TRIANGLE_HEIGHT
	x = int(x)
	y = robot_y + np.sin(rad)*TRIANGLE_HEIGHT
	y = int(y)
	pt1 = (x,y)
	
	rad = np.radians(robot_theta+120)
	x = robot_x + np.cos(rad)*TRIANGLE_HEIGHT
	x = int(x)
	y = robot_y + np.sin(rad)*TRIANGLE_HEIGHT
	y = int(y)
	pt2 = (x,y)
	
	rad = np.radians(robot_theta+240)
	x = robot_x + np.cos(rad)*TRIANGLE_HEIGHT
	x = int(x)
	y = robot_y + np.sin(rad)*TRIANGLE_HEIGHT
	y = int(y)
	pt3 = (x,y)

	triangle_cnt = np.array( [pt1, pt2, pt3] )

	cv2.drawContours(this_map, [triangle_cnt], 0, (0,0,0), -1)
	cv2.circle(this_map, pt1, 1, (0,0,255), -1)


def drawRobot(this_map):
		
	global loc
		
	num_display = 100
	for max_pos in range(1):
			for i in range(MAX_OBS-1):
				if not MAPPING:
					break
				mean = obs_mean[max_pos][i]
				mean_y = (mean[0]).astype('int')
				mean_x = (mean[1]).astype('int')
			
				var = obs_var[max_pos][i]
				var_y = var[0]
				var_x = var[1]

				std_y = (var_y**.5).astype('int')
				std_x = (var_x**.5).astype('int')
					
				if mean_x != -1:	
					cv2.rectangle(this_map,(mean_x-25,mean_y-25),(mean_x+25,mean_y+25),(0,0,255),-1)

	y = loc[:,0].astype('int')
	y[y>=600] = 599
	y[y<0] = 0
	x = loc[:,1].astype('int')
	x[x>=300] = 299
	x[x<0] = 0
	this_map[y,x] = [255,0,255]
	drawRobotTriangle(this_map,loc[0,1],loc[0,0],loc[0,2])			
	
def drawMap():

	this_map = soc_map.copy()
	
	drawRobot(this_map)
	
	return this_map	

def showIm():
	while True:
		currIm = nextIm
		cv2.imshow('img',currIm)

		this_map = drawMap()
		cv2.imshow('map',this_map)
    	
		key = cv2.waitKey(30)
    		if key == 27:#if ESC is pressed, exit loop
        		cv2.destroyAllWindows()
       	 		break


# why did this make everything so slow
def applyMotion(move,turn):

	global loc	
	
	angular_move = 0		
	if turn == 'left':
		angular_move = -LINEAR_MOVEMENT_PER_RATE
	elif turn == 'right':
		angular_move = LINEAR_MOVEMENT_PER_RATE
	
	linear_move = 0
	if move:
		linear_move = LINEAR_MOVEMENT_PER_RATE
	
	rate = np.random.normal(0,MOVEMENT_STD,(PARTICLES,2))
	if imFeatures['Homerun']:
		rate[:] = 0
	rate[:,0] += linear_move
	rate[:,1] += angular_move
		

	loc[:,2] += rate[:,1]
	loc[:,2] = loc[:,2]%360
	loc[:,0] += np.sin(np.radians(loc[:,2]))*rate[:,0]
	loc[:,1] += np.cos(np.radians(loc[:,2]))*rate[:,0]
	


def distance(point1,point2 ):
	dist_value = math.dist(point1,point2)


def offTheEdge():
	
	global loc

	edge = loc[:,1]
	bigger =  edge < 350
	smaller =  edge > -50
	edge = np.logical_and(bigger,smaller)	
	
	oedge = loc[:,0]
	bigger =  oedge < 600
	smaller =  oedge > 0
	oedge = np.logical_and(bigger,smaller)	
	
	edge = np.logical_and(edge,oedge).astype('float')

	return edge

def goal_dist_prob(other = False):
	
	y_value = loc[:,0].copy()

	if other:
		y_value = 600 - y_value

	if other:
		visible_goal = np.ones(PARTICLES)*int(imFeatures['Other Goal Ahead'])
		close_goal = np.ones(PARTICLES)*int(imFeatures['Homerun'])
		if imFeatures['Other Goal Dist'] == 'none':
			return np.ones(PARTICLES)
		else:
			goal_dist = np.ones(PARTICLES)*int(imFeatures['Other Goal Dist'])
	else:
		visible_goal = np.ones(PARTICLES)*int(imFeatures['Goal Ahead'])
		close_goal = np.ones(PARTICLES)*int(imFeatures['Homerun'])
		if imFeatures['Goal Dist'] == 'none':
			return np.ones(PARTICLES)
		else:
			goal_dist = np.ones(PARTICLES)*int(imFeatures['Goal Dist'])
			

	dist_prob = gaussian_prob(y_value,goal_dist,10)

	output_prob = np.maximum((1-visible_goal),(dist_prob))
	output_prob = np.maximum((1-close_goal),(output_prob))

	return output_prob
	

def goal(other = False):
	
	x_value = loc[:,1]
	y_value = loc[:,0]
		
	angle = loc[:,2]
	angle = np.radians(angle)
	
	# equation of a line is y = mx + b 
	slope = np.tan(angle)
	intercept_b = y_value -(slope * x_value)

	if other:
		x_intercept = (600-intercept_b) / (slope)
	else:	
		x_intercept = -1 * intercept_b / (slope)

	if other:
		visible_goal = np.ones(PARTICLES)*int(imFeatures['Other Goal Ahead'])
		obstacle_ahead = np.ones(PARTICLES)*int(imFeatures['Obstructed'])
		close_goal = np.ones(PARTICLES)*int(imFeatures['Homerun'])
		right = np.ones(PARTICLES)*int(imFeatures['Other Goal Direction'] == 'right')#left is not right
		goal_pos = np.ones(PARTICLES)*float(imFeatures['Other Goal Pos'])
		goal_pos = 1 - goal_pos
		goal_occluded = imFeatures['Other Goal Occluded']

	else:	
		visible_goal = np.ones(PARTICLES)*int(imFeatures['Goal Ahead'])
		obstacle_ahead = np.ones(PARTICLES)*int(imFeatures['Obstructed'])
		close_goal = np.ones(PARTICLES)*int(imFeatures['Homerun'])
		right = np.ones(PARTICLES)*int(imFeatures['Goal Direction'] == 'right')#left is not right
		goal_pos = np.ones(PARTICLES)*float(imFeatures['Goal Pos'])
		goal_occluded = imFeatures['Goal Occluded']
	
	goal_intercept_position = (x_intercept - 60).astype('float')/float(240-60)
	left = 1*(np.logical_and((goal_intercept_position > 0), (goal_intercept_position < .5)))
	right = 1*(np.logical_and((goal_intercept_position >= .5), (goal_intercept_position < 1.0)))
	neither = 1*np.logical_not(np.logical_or(left,right))
	facing_forward = np.ones(PARTICLES)*(np.logical_and((angle > 0),(angle < np.math.pi)))
	if not other:
		facing_forward = 1 - facing_forward	

	# Do I expect to see the goal?
	expect_goal = np.logical_not(neither)
	expect_goal = 1*np.logical_and(expect_goal,facing_forward)
	
	expect_right = 1*np.logical_and(right,facing_forward)
	correct_pos = 1*np.logical_and(goal_pos > goal_intercept_position - (GOAL_DIR_THRESH)/(180.0), 
					goal_pos < goal_intercept_position + GOAL_DIR_THRESH/(180.0))


	# we do not see the goal and do not expect to see it
	# might want to break up into two
	scenario_1 = 1*np.logical_and(np.logical_not(expect_goal),
				 	np.logical_not(visible_goal))

	scenario_1_prob = np.ones(PARTICLES)

	if goal_occluded:
		scenario_2 = 1*np.logical_and(expect_goal,visible_goal)
	else:
		scenario_2 = 1*np.logical_and(np.logical_and(expect_goal,visible_goal),
					correct_pos)
	
	scenario_2_prob = np.ones(PARTICLES)

	# We see the goal but we are not facing forward
	scenario_3 = 1*np.logical_and(visible_goal,np.logical_not(facing_forward))
	
	scenario_3_prob = np.zeros(PARTICLES)
	
	# We see the goal and are facing forward but not towards the goal
	scenario_4 = 1*np.logical_and(visible_goal,
					np.logical_not(expect_goal))

	# This was an expriment 
	# it worked but its too much work for the benefits	
	#left_prob = 1-(60-x_intercept)/GOAL_FUZZ
	#left_prob[left_prob > 1] = 0
	#left_prob[left_prob < 0] = 0
	#right_prob = 1-(x_intercept-240)/GOAL_FUZZ
	#right_prob[right_prob > 1] = 0
	#right_prob[right_prob < 0] = 0
	#scenario_4_prob = np.maximum(left_prob,right_prob)
	
	scenario_4_prob = np.zeros(PARTICLES)
	
	# we see the goal and expect to see it but the direction is wrong
	# might want to break up into two	

	output_prob = (scenario_1*scenario_1_prob + 
			scenario_2*scenario_2_prob + 
			scenario_3*scenario_3_prob + 
			scenario_4*scenario_4_prob)


	output_prob = np.maximum(output_prob,obstacle_ahead)
	output_prob = np.maximum(close_goal,output_prob)

	output_prob = np.maximum(output_prob,np.ones(PARTICLES)*.2)
	
	return output_prob

def offFieldTraj():

	x_value = loc[:,1]
	y_value = loc[:,0]
		
	angle = loc[:,2]
	angle = np.radians(angle)
	
	slope = np.tan(angle)
	intercept_b = y_value -(slope * x_value)

	x_intercept = -1 * intercept_b / (slope)
	left_yValue = (slope * -50 ) + intercept_b
	right_yValue = (slope * 350) + intercept_b
	 
	bounding_field_left  = 1*(np.logical_and((left_yValue >0 ), (left_yValue < 600)))
	bounding_field_right = 1 * (np.logical_and((right_yValue > 0),(right_yValue < 600)))

	looking_left = np.logical_and(angle > np.math.pi/2, angle < 3*np.math.pi/2)
	looking_right = np.logical_not(looking_left)
	
	robot_left = np.logical_and(x_value >= -50,x_value < 150)
	robot_right = np.logical_and(x_value >= 150,x_value <= 350)

	
	if imFeatures['Soft EOF Line Ahead'] == None:

		robot_outfield = np.zeros(PARTICLES) 
	else: 
		robot_outfield = np.ones(PARTICLES)
		
	obstacle_ahead = np.ones(PARTICLES)*int(imFeatures['Obstructed'])
	close_goal = np.ones(PARTICLES)*int(imFeatures['Homerun'])

	# if it is left and it is looking left  and there is an intersection it should see an eof
	left_left = looking_left*robot_left
	left_left_prob = bounding_field_left == robot_outfield
	left_left_prob = left_left*left_left_prob
	
	# if it is right and it is looking right, and there is an intersection it should see an eof
	right_right = looking_right*robot_right
	right_right_prob = bounding_field_right == robot_outfield
	right_right_prob = right_right*right_right_prob
	
	right_left = looking_right*robot_left
	right_left_prob = bounding_field_right == robot_outfield
	right_left_prob = right_left*right_left_prob
	
	left_right = looking_left*robot_right
	left_right_prob = bounding_field_left == robot_outfield
	left_right_prob = left_right*left_right_prob

	care = left_right+right_left+left_left+right_right
	care = care*(left_right_prob+right_left_prob+left_left_prob+right_right_prob)

	# if it is neither of these situations, forget about it (set it to 1)
	neither = np.logical_or(np.logical_and(robot_right,looking_left),np.logical_and(robot_left,looking_right))

	# if it is obstructed, forget about it
	dont_care = np.logical_or(close_goal,obstacle_ahead)

	out = np.maximum(dont_care,care)

	# ... this is not the most reliable lol, never let it completly kill a particle
	out = np.maximum(out,np.ones(PARTICLES)*.2)

	return out

def observe():

	global loc
	global imFeatures

	visibleGoalAhead = goal()
	ovisibleGoalAhead = goal(other=True)
	goal_dist_ = goal_dist_prob()
	ogoal_dist_ = goal_dist_prob(other = True)
	edge = offTheEdge()
	offField = offFieldTraj()
	offField = 1
	prob = (edge*goal_dist_*ogoal_dist_
		*ovisibleGoalAhead*visibleGoalAhead*offField)

	
	global observe_prob	
	observe_prob =  prob

def resampling():

	global loc
	global obs_mean
	global obs_var
	global  obs_num
	global obs_used 

	cumsum = loc[:,3].cumsum()
	total_weight = cumsum[-1]
	step_length = total_weight/PARTICLES

	# pick a random starting location
	start = np.random.uniform(0,step_length)
	
	# calculate how many particles each particle spawns
	before = (np.concatenate([np.zeros((1)),cumsum[:-1]],axis = 0)+start)//step_length
	before = before.astype('int')
	after = (cumsum+start)//step_length
	after = after.astype('int')
	num_sample = after - before

	# remove 0 particles and multiply >0 particles
	loc = np.repeat(loc,num_sample,axis = 0)
	obs_mean = np.repeat(obs_mean,num_sample,axis = 0)
	obs_var = np.repeat(obs_var,num_sample,axis = 0)
	obs_num = np.repeat(obs_num,num_sample,axis = 0)
	obs_used = np.repeat(obs_used,num_sample,axis = 0)

def obs_distance(obs):

	p = float(obs[3])
	w = 50.0
	f = 510.0
	
	dist = (w*f)/p

	return dist + 25

def obs_angle(obs):

	# get the center and convert to a coordinate system where the robots trajectory is the positive y axis
	obs_center_x = obs[0]+obs[2]//2
	obs_center_x -= nextIm.shape[1]//2
	obs_center_y = obs[1]+obs[3]//2
	obs_center_y -= nextIm.shape[0]
	obs_center_y *= -1 

	angle = np.degrees(np.arctan2(obs_center_x,obs_center_y))
	
	correction = 40.0
	correction = correction/90.0
	correction = 1 - correction
	
	angle = float(angle)*correction
	
	return angle

def sameObstacle(x,y):
	
	global obs_mean
	global obs_var
	global obs_num
	global obs_used

	# reshape to the number of obstacles
	y_rep = y.reshape((PARTICLES,1))
	y_rep = np.repeat(y_rep,MAX_OBS-1,axis = 1)
	x_rep = x.reshape((PARTICLES,1))
	x_rep = np.repeat(x_rep,MAX_OBS-1,axis = 1)
	
	yx = np.stack([y_rep,x_rep],axis = -1)

	dist = yx - obs_mean[:,:-1]
	dist = dist[:,:,0]**2 + dist[:,:,1]**2
	dist = dist**.5
	dist = dist*(1-obs_used[:,:-1]) + 1e10*obs_used[:,:-1]


	euc_closest = np.argmin(dist,axis = -1).reshape((PARTICLES,1))
	euc_close = dist<75
	euc_in_dist = (euc_close.sum(axis=-1) > 0).reshape((PARTICLES,1))

	# get the probability of the observation x,y 
	#under each of the obstacles' gaussian distribiution
	prob = gaussian_prob(np.stack([y_rep,x_rep],axis = -1)
				,np.stack([obs_mean[:,:-1,0],obs_mean[:,:-1,1]],axis = -1)
				,np.stack([obs_var[:,:-1,0]+1e-7,obs_var[:,:-1,1]+1e-7],axis = -1))
	
	# multiply the x and y probablities
	prob = prob[...,0]*prob[...,1]

	# remove obstacles that have already been used
	prob = prob*obs_used[:,:-1]

	# check how many of them are greater then the minimum probablity
	closest = np.argmax(prob,axis = -1).reshape((PARTICLES,1))
	close = prob>MIN_PROB
	
	close = (1-euc_in_dist)*close + euc_close*euc_in_dist
	closest = (1-euc_in_dist)*closest + euc_closest*euc_in_dist
	in_dist = (close.sum(axis=-1) > 0).reshape((PARTICLES,1))
	
	close = close[:,0]
	closest = closest[:,0]
	in_dist = in_dist[:,0]
	euc_close = euc_close[:,0]
	euc_closest = euc_closest[:,0]
	euc_in_dist = euc_in_dist[:,0]

	# check if the observation falls out of the map
	noise = np.logical_or(np.logical_or(y < 0,y > 600),
			      np.logical_or(x < 0,x > 300))
	
	# increment the number of obstacles for the observations
	# where none of the of the obstacles explain the observation well enough
	# and it is not noise 
	new_obs_num = obs_num + 1	
	new_obs_num = np.minimum(new_obs_num,MAX_OBS)
	obs_num = new_obs_num*(1-in_dist)*(1-noise) + obs_num*(in_dist+noise)
	
	# get what those probablities are
	best_prob = prob[np.arange(PARTICLES),closest]*(1-euc_in_dist) + np.ones(PARTICLES)*euc_in_dist*1e-3
	
	# return the index of the obstacle that we are that the observation belong too
	out = (closest*in_dist*(1-noise) 
	      + (new_obs_num-1)*(1-in_dist)*(1-noise) 
	      + np.ones(PARTICLES)*(MAX_OBS-1)*noise)
	
	# get those probabilityes
	best_prob = (best_prob*in_dist*(1-noise) 
      			+ np.ones(PARTICLES)*(1-in_dist)*(1-noise)*1e-8
	      		+ np.zeros(PARTICLES)*noise)
		

	return out.astype('int'),best_prob
	


def updateLandmarks():
	
	global imFeatures
	global loc
	global obs_mean
	global obs_var
	global landmark_prob	
	global obs_used


	obs_used[:] = 1
	obs = imFeatures['All Obstacles']
	obstructed = obs != None
	if obstructed:
		obstructed = len(obs) != 0

	if not obstructed:
		landmark_prob = np.ones(PARTICLES)
		return

	rolling_prob = np.ones(PARTICLES)
	for i in range(len(obs)):
		if i == MAX_OBS:
			break

		ob = obs[i]
		dist = obs_distance(ob)
		angle = obs_angle(ob)
	
		if dist == 'none':
			break

		y =  loc[:,0]
		x =  loc[:,1]
		t =  loc[:,2]+angle			
	
		next_y_mean = y + np.sin(np.radians(t))*dist
		next_x_mean = x + np.cos(np.radians(t))*dist

		next_y_var = np.ones(PARTICLES)*OBS_VAR
		next_x_var = np.ones(PARTICLES)*OBS_VAR
		
		closest,prob = sameObstacle(next_x_mean,next_y_mean)
		obs_used[np.arange(PARTICLES),closest] = 0
	
		prev_y_mean = obs_mean[np.arange(PARTICLES),closest,0]
		prev_x_mean = obs_mean[np.arange(PARTICLES),closest,1]	
			
		prev_y_var = obs_var[np.arange(PARTICLES),closest,0]
		prev_x_var = obs_var[np.arange(PARTICLES),closest,1]
	
		prev_y_mean[prev_y_mean == -1] = next_y_mean[prev_y_mean == -1]
		prev_x_mean[prev_x_mean == -1] = next_x_mean[prev_x_mean == -1]
		
		prev_y_var[prev_y_var == 0] = OBS_VAR
		prev_x_var[prev_x_var == 0] = OBS_VAR

		k_x = prev_x_var*(1/(prev_x_var+next_x_var))
		k_y = prev_y_var*(1/(prev_y_var+next_y_var))

		x_mean_prime = prev_x_mean + k_x*(next_x_mean - prev_x_mean)
		y_mean_prime = prev_y_mean + k_y*(next_y_mean - prev_y_mean)

		x_var_prime = prev_x_var - k_x*prev_x_var
		y_var_prime = prev_y_var - k_y*prev_y_var

		obs_mean[np.arange(PARTICLES),closest,0] = y_mean_prime
		obs_mean[np.arange(PARTICLES),closest,1] = x_mean_prime	
			
		obs_var[np.arange(PARTICLES),closest,0] = y_var_prime
		obs_var[np.arange(PARTICLES),closest,1] = x_var_prime
		
		rolling_prob *= prob
	
	landmark_prob = rolling_prob

landmark_prob = None
observe_prob = None
make_observation = False

def infObserve():
	global make_observation
	while True:
		while not make_observation:
			pass
		observe()
		make_observation = False

observeThread = threading.Thread(target=infObserve)
observeThread.start()

make_landmark = False
def infLandmark():
	global make_landmark
	while True:
		while not make_landmark:
			pass
		updateLandmarks()
		make_landmark = False	
landmarkThread = threading.Thread(target=infLandmark)
landmarkThread.start()



skip = 1
def particleFilter(move,turn):

	global loc
	global make_observation
	global make_landmark
	global skip	
	global observe_prob
	global landmark_prob

	applyMotion(move,turn)
	
	if skip == 1:
		make_observation = True
	
		if MAPPING:
			make_landmark = True

		while make_observation:
			pass
		while make_landmark:
			pass
		
		#landmark_prob[landmark_prob < 1e-20] = 1e-20
		#landmark_prob = landmark_prob/landmark_prob.sum()
		
		#observe_prob[observe_prob < 1e-20] = 1e-20
		#observe_prob = observe_prob/observe_prob.sum()
	
		prob = observe_prob 
		if MAPPING:
			prob += landmark_prob
		prob[prob < 1e-20] = 1e-20
		prob = prob/prob.sum()

		loc[:,3] = prob
			
		resampling()
		skip = 0
	else:
		skip += 1	
	
	

def controlRobot():
	while True:
		twist.angular.z = angularZ
    		twist.linear.x=linearX
    		robot.publish_twist(twist)

		move = (linearX != 0)
		turn = None
		if angularZ > 0:
			turn = 'left'	
		elif angularZ < 0:
			turn = 'right'
		
		particleFilter(move,turn)

		rate.sleep()	


def moveRobot(move,turn):

	global angularZ
	global linearX
	
	if move: 
    		newLinearX = X_MOVE_SPEED
	else:
    		newLinearX = 0

	if turn == 'left':
		newAngularZ = radians(ANGULAR_MOVE_SPEED)
	elif turn == 'right':
		newAngularZ = radians(-ANGULAR_MOVE_SPEED)
	else:
		newAngularZ = 0

	angularZ = newAngularZ
	linearX = newLinearX

def goal_occluded(frame,goal,obs):
	occluded = False
	if goal[0]==0:
		occluded = True
	if goal[0]+goal[2] == frame.shape[2]:
		occluded = True
	if obs != None:
		for ob in obs:
			if (ob[0] - goal[0]) < goal[2]:
				occluded = True
			if (goal[0] - ob[0]) < ob[2]:
				occluded = True
	return occluded


def goal_distance(frame,goal):

	p = float(goal[3])
	if p < 65:
		return 'none'
	f = 518.0
	w = 80.0

	dist = (w*f)/p
	
	return dist

skip_frames = 300
def getIm():
	
	frame = robot.get_image()
	
	image = frame.copy()
	obs = findObstacle(frame)
	goal = findGoal(frame)
	otherGoal = findGoal(frame,True)
	image = drawGoal(image,goal)
	image = drawGoal(image,otherGoal)
	lines = findLines(frame,obs,goal)   
	image = drawLines(image,lines)
	image = drawLegend(image)
	allObs = findObstacle(frame,removeSmall = False)
	image = drawObstacle(image,allObs)
		
	global nextIm
	nextIm = image
	
	imFeatures['All Obstacles'] = allObs


	imFeatures['Obstacles'] = obstruction(frame,obs) 
	imFeatures['EOF Line Ahead'] = EOFInFront(frame,lines)
	imFeatures['Soft EOF Line Ahead'] = EOFInFront(frame,lines,True)
	imFeatures['Goal Ahead'] = goalInFront(frame,goal)
	imFeatures['Goal Pos'] = goalInFront(frame,goal,exact = True)
	imFeatures['Goal Occluded'] = goal_occluded(frame,goal,obs)
	imFeatures['Goal Dist'] = goal_distance(frame,goal)
	global skip_frames
	if skip_frames > 0:
		skip_frames -= 1
	else:
		imFeatures['Other Goal Ahead'] = goalInFront(frame,otherGoal)
		imFeatures['Other Goal Direction'] = goalDir(frame,otherGoal)
		imFeatures['Other Goal Pos'] = goalInFront(frame,otherGoal,exact = True)
		imFeatures['Other Goal Occluded'] = goal_occluded(frame,otherGoal,obs)
		imFeatures['Other Goal Dist'] = goal_distance(frame,otherGoal)
	imFeatures['Obstructed'] = (imFeatures['Obstacles'] != None)
	imFeatures['Big Goal'] = goalHo(frame,goal,BIG_GOAL_THRESH)
	imFeatures['Goal Direction'] = goalDir(frame,goal)
	imFeatures['Obs Direction'] = obsDir(frame,imFeatures['Obstacles']) 	
	imFeatures['Big Obs'] = obsHo(frame,imFeatures['Obstacles'],BIG_OBS_THRESH)
	imFeatures['Goal In Sight'] = goalInSight(goal,frame)
	imFeatures['Other Goal In Sight'] = goalInSight(otherGoal,frame)

def initialState():

	# Decide what the first state is 
	nextState = pointTowardsGoal

	# Procced to the next state
	return nextState

def moveTowardsGoal():

	if STATE_PRINT:
		print("State:Move Towards Goal")	

	move = True
	di = None
	if imFeatures['Goal Ahead']:
		move = True
	elif imFeatures['Goal Direction'] == 'left':
		di = 'left'
	elif imFeatures['Goal Direction'] == 'right':
		di = 'right'

	moveRobot(move,di)

	# get the next image
	getIm()

	# decide what to do next
	if not imFeatures['Goal Ahead']:
		nextState = pointTowardsGoal
	elif imFeatures['Big Goal']:
		nextState = homerun
	else:
		nextState = moveTowardsGoal

	# Procced to next state
	return nextState


def pointTowardsGoal():

	if STATE_PRINT:
		print("State:Point Towards Goal")	

	move = False
	di = imFeatures['Goal Direction']
	if not (imFeatures['Big Obs'] or not imFeatures['Goal In Sight']) :
		move = True

	moveRobot(move,di)

	# get the next image
	getIm()
	
	# decide what to do next
	if not imFeatures['Goal Ahead']:
		if imFeatures['Times Looking For Goal'] == 0:
			nextState = pointAwayFromObs
			print("!!!!!!!!!!!!!!!!!BREAK DEADLOCK!!!!!!!!!!!!!!!!!")
			imFeatures['Times Looking For Goal'] = DEADLOCK_BREAK
		else:
			nextState = pointTowardsGoal
			imFeatures['Times Looking For Goal'] -= 1
	elif imFeatures['Obstructed']:
		nextState = pointAwayFromObs
		imFeatures['Times Looking For Goal'] = DEADLOCK_BREAK
	elif imFeatures['Big Goal']:
		nextState = homerun
		imFeatures['Times Looking For Goal'] = DEADLOCK_BREAK
	else:
		nextState = moveTowardsGoal
		imFeatures['Times Looking For Goal'] = DEADLOCK_BREAK

	# Procced to next state
	return nextState
	

def pointAwayFromObs():

	if STATE_PRINT:
		print("State:Point Away From Obstacle")	
	
	move = False
	di = imFeatures['Obs Direction']		
	if not imFeatures['Big Obs']:
		move = True

	moveRobot(move,di)

	# get the next image
	getIm()
	
	# decide what to do next
	if imFeatures['Obstructed']:
		if len(imFeatures['Obstacles']) > 1:
			imFeatures['Num Move Past'] = TWIST_PAST_CLUSTER_NUM
			nextState = pointAwayFromManyObs()
			
		else:
			nextState = pointAwayFromObs
	else:	
		imFeatures['Num Move Past'] = MOVE_PAST_OBS_NUM
		nextState = movePastObs

	return nextState

def pointAwayFromEOF():

	if True:
		print("State:Point Away From EOF")	
	
	move = False

	moveRobot(move,'left')

	# get the next image
	getIm()
	
	# decide what to do next
	if imFeatures['EOF Line Ahead'] != None:
		nextState = pointAwayFromEOF
	else:	
		nextState = movePastObs

	return nextState

def pointAwayFromManyObs():
	
	if STATE_PRINT:
		print("State:Point Away From Many Obs")
	
	imFeatures['Num Move Past'] -= 1
	
	moveRobot(False,'right')
		
	# get the next image
	getIm()
	
	# decide what to do next
	if imFeatures['Num Move Past'] < 0:
		nextState = pointAwayFromObs
	else:
		nextState = pointAwayFromManyObs
	
	return	nextState

def movePastObs():
	
	if STATE_PRINT:
		print("State:Move Past Obsticle")
	
	imFeatures['Num Move Past'] -= 1

	move = True
	di = imFeatures['Obs Direction']

	moveRobot(move,di)	
		
	# get the next image
	getIm()
	
	# decide what to do next
	if imFeatures['Num Move Past'] < 0:
		nextState = pointTowardsGoal
	elif imFeatures['EOF Line Ahead'] == None:
		nextState = movePastObs
	else:
		nextState = pointAwayFromEOF
	
	return	nextState

def movePastEOF():
	
	if TRUE:
		print("State:Move Past EOF")
	
	imFeatures['Num Move Past'] -= 1

	move = True

	moveRobot(move,None)	
		
	# get the next image
	getIm()
	
	# decide what to do next
	if imFeatures['Num Move Past'] < 0:
		nextState = pointTowardsGoal
	else:
		nextState = movePastEOF
	
	return	nextState

def homerun():
	
	if STATE_PRINT:
		print("State:HOMERUN!!!!")	

	imFeatures['Homerun'] = True
	
	# move forward a little bit
	moveRobot(True,None)

	# get the next image
	getIm()

	# Always come back to homerun
	if True:
		nextState = homerun

	# Procced to next state
	return nextState
	

if __name__ == '__main__':
	

	imgThread = threading.Thread(target=showIm)
	imgThread.start()
	
	robThread = threading.Thread(target=controlRobot)
	robThread.start()

	nextState = initialState
	i = 0
	while True:
		rate.sleep()
		nextState = nextState()
		i+=1	













