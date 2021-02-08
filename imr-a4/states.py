def initialState():

	# Decide what the first state is 
	nextState = pointTowardsGoal

	# Procced to the next state
	return nextState

def moveTowardsGoal():

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
		nextState = pointTowardsGoal
	elif imFeatures['Obstructed']:
		nextState = pointAwayFromObs
		imFeatures['Steady Obs Direction']  = imFeatures['Obs Direction']
	elif imFeatures['Big Goal']:
		nextState = homerun
	else:
		nextState = moveTowardsGoal

	# Procced to next state
	return nextState
	

def pointAwayFromObs():

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

def pointAwayFromManyObs():
	
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
	else:
		nextState = movePastObs
	
	return	nextState

def homerun():
	
	print("State:HOMERUN!!!!")	
	
	# move forward a little bit
	moveRobot(True,None)

	# get the next image
	getIm()

	# Always come back to homerun
	if True:
		nextState = homerun

	# Procced to next state
	return nextState
