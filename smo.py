from myro import *

import math
import time
INITIALIZE_NUMBER = "COM40"
initialize(INITIALIZE_NUMBER)
autoCamera()

TURNS = 24
N = TURNS/2
baseline = 0

dTHETA = 2 * math.pi / TURNS
lStack = []
rStack = []

#find out how many turns of dTHETA it takes to turn PI radians, call that N
#to do this, we use the obstacle sensors and place two relatively narrow but tall obstacles for the obstacle sensors to detect, on either side of the robot
#there must be no other obstacles around the robot to begin with
flag = 0
firstEncounter = 0
lastEncounter = 0

motors(1,1)
wait(0.25)
motors(-1,-1)
wait(0.25)
motors(0,0)

for i in xrange(TURNS):
	bright = getBright()
	for j in xrange(3):
		##print "average brightness:",bright[j]
		baseline += bright[j] #determine baseline brightness over 2N rotations of dTHETA
	reading = getObstacle(1)
	##print(reading)
	'''
	if (reading > 900 and firstEncounter == 0):
		firstEncounter = i
		lastEncounter = i
	elif (reading > 900 and firstEncounter != 0):
		lastEncounter = i
	'''
	turnL(dTHETA, 1)
#N = int((firstEncounter + lastEncounter) / 2)
baseline /= (TURNS)*3 #baseline is now the average brightness encountered during the robot's approx 2PI sweep
#now when we want to turn PI around we perform N turns of dTHETA	
##print "baseline1:",baseline
beep(0.5, 440, 659)
##print(N)


#determine location of brightest light source
brightness = 0
maxBrightness = -1
maxPos = 0
for i in xrange(TURNS):
	bright = getBright()
	brightness = 0
	for j in xrange(3):
		brightness += bright[j]
	##print("brightness:",brightness)
	if (brightness > maxBrightness):
		maxBrightness = brightness
		maxPos = i
	turnL(dTHETA, 1)

for i in xrange(TURNS - maxPos):
	turnR(dTHETA, 1)

#start the rolling input for left, right readings
#with the rolling input, new readings degrade in effect over time while still maintaining some influence over movement
FACTOR = 0.05
roller = [0, 0, 0]
stage = 0
leftInput = 0
rightInput = 0
left = [0]
right = [0]
baselineCount = 0
MAX_BASELINE_COMBO = 5
BASELINE_THRESHOLD = 50000
average = 0
WAIT_TIME = 0.53
##print("Baseline is " + str(baseline))
while (stage == 0):
	ts = time.time()
	bright = getBright()
	average = 0
	for i in xrange(3):
		roller[i] *= FACTOR
		roller[i] += bright[i]
		average += bright[i]
		##print "bright #",i,"=",bright[i]
	average /= 3
	##print("Average reading is", average)
	#follow light source using rolling input to sigmoid function
	leftInput = sigmoidFcn(bright[2] - bright[0] + bright[1])
	rightInput = sigmoidFcn(bright[0] - bright[2] + bright[1])
	#each movement command stored on a stack to be reversed later
	left.append(leftInput)
	right.append(rightInput)
	motors(leftInput, rightInput)
	#wait(WAIT_TIME)
	#when average brightness reading approaches baseline, begin reverse operation
	if (average < BASELINE_THRESHOLD + baseline):
		baselineCount += 1
		#print(baselineCount)
	else:
		baselineCount = 0
	if (baselineCount == MAX_BASELINE_COMBO):
		stage = 1
	#print time.time()-ts
##print(left)
##print(right)
motors(0, 0)
wait(0.1)
beep(0.5, 523, 698)
wait(0.5)
for i in xrange(N):
	turnL(dTHETA, 1)
while (len(left) > 0 and len(right) > 0):
	ts = time.time()
	R = right.pop()
	L = left.pop()
	##print L, R
	motors(R, L)
	wait(WAIT_TIME)
	#print(time.time() - ts)

beep(1.0, 523, 1046)
motors(0, 0)

K = 200000.0

def sigmoidFcn(brightness):
	return(1.0/(1.0+math.exp(-brightness/K)))

'''
TURNING FUNCTION USING ANGLES(RADIANS)
- take in degrees in radians and either speed(0-1) or time to turn
- Affected by friction so angle will be inaccurate
'''
def turnL(degree, speed):
    turnLeft(speed, degree/(2.0*speed))
def turnR(degree, speed):
    turnRight(speed, degree/(2.0*speed))