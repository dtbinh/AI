# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import vrep, math, time, env, random


MOTOR_RIGHT = 1;
MOTOR_LEFT = 2;
MOTOR_REVERSE = 3
MOTOR_FRONT = 4


# get distance to obstacle detected by a given sensor
def getObstacleDist(sensorHandler_):
    dist2Obstacle_LR = [0.0, 0.0]
    # Get raw sensor readings using API
    rawSR = vrep.simxReadProximitySensor(clientID, sensorHandler_, vrep.simx_opmode_oneshot_wait)
    print(rawSR)
    # Calculate Euclidean distance
    if rawSR[1]: # if true, obstacle is within detection range, return distance to obstacle
        return math.sqrt(rawSR[2][0]*rawSR[2][0] + rawSR[2][1]*rawSR[2][1] + rawSR[2][2]*rawSR[2][2])
    else: # if false, obstacle out of detection range, return inf.
        return float('inf')

# agent's reasoning procedure, using IF-THEN rules
def getMotorSpeed(dist):
    # You can implement your navigation strategy here

    return motorSpeed

# execute agent's action
def execute(motorSpeed):
    vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeed['speedLeft'], vrep.simx_opmode_oneshot )
    vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeed['speedRight'], vrep.simx_opmode_oneshot )

def runPioneerToAngle(goto_angle):
    abs_angle = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)

    hysteres = 0.15

    while abs_angle[1][2] > (goto_angle + hysteres) or abs_angle[1][2] < (goto_angle - hysteres):

        run_motor(MOTOR_RIGHT, MID_SPEED, SHORT_TIME, 0)
        abs_angle = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)


def calculateRedBlockAngle():

    import math

    dist2NearestBlock, blockId, pioneer2BlockPos = env.getNearestConcretBlockDist(clientID, blocksArray, pioneerRobotHandle)


    #ret, Coordinates = vrep.simxGetObjectPosition(clientID, pioneerRobotHandle, blockId, vrep.simx_opmode_oneshot)

    x = pioneer2BlockPos[1][0]
    y = pioneer2BlockPos[1][1]

    angle = math.atan2(y,x)

    if angle > math.pi:
        angle = angle - math.pi*2

    elif angle < math.pi*(-1) :
        angle = angle + math.pi*2

    if angle < 0:
        angle = angle + math.pi
    else:
        angle = angle - math.pi

    return angle

def run_motor(LOCATION, speed, sleeptime, recoverTime):

    import time

    if LOCATION == MOTOR_RIGHT :
        motorSpeed = dict(speedLeft=speed, speedRight=(-1)*speed)
        execute(motorSpeed)
        time.sleep(sleeptime)

        motorSpeed = dict(speedLeft=0, speedRight=0)
        execute(motorSpeed)
        time.sleep(recoverTime)

    elif LOCATION == MOTOR_LEFT :
        motorSpeed = dict(speedLeft=(-1)*speed, speedRight=speed)
        execute(motorSpeed)
        time.sleep(sleeptime)

        motorSpeed = dict(speedLeft=0, speedRight=0)
        execute(motorSpeed)
        time.sleep(recoverTime)

    elif LOCATION == MOTOR_REVERSE :
        motorSpeed = dict(speedLeft=(-1)*speed, speedRight=(-1)*speed)
        execute(motorSpeed)
        time.sleep(sleeptime)

    elif LOCATION == MOTOR_FRONT :
        motorSpeed = dict(speedLeft=speed, speedRight=speed)
        execute(motorSpeed)
        time.sleep(sleeptime)









print 'Program started'
vrep.simxFinish(-1) # just in case, close all opened connections

int_portNb = 19999 # define port_nr
clientID = vrep.simxStart( '127.0.0.1', int_portNb, True, True, 5000, 5) # connect to server
if clientID != -1:
    print 'Connected to remote API server'
    res,objs = vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait) # get all objects in the scene
    if res == vrep.simx_return_ok: # Remote function call succeeded
        print 'Number of objects in the scene: ',len(objs)# print number of object in the scene

        ret_lm,  leftMotorHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        ret_rm,  rightMotorHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        ret_pr,  pioneerRobotHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)


        #USS =  UltraSonicSensor

        #FRONT SENSORS
        ret_sl,  USS_LF_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(1),vrep.simx_opmode_oneshot_wait) #max left
        ret_sl,  USS_LF_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(2),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_LF_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(3),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_LF_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(4),vrep.simx_opmode_oneshot_wait) #centrum
        ret_sl,  USS_RF_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(5),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(6),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(7),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(8),vrep.simx_opmode_oneshot_wait) #max right

        #BACK SENSORS
        ret_sl,  USS_RB_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(9),vrep.simx_opmode_oneshot_wait) #max right
        ret_sl,  USS_RB_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(10),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_RB_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(11),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_RB_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(12),vrep.simx_opmode_oneshot_wait) #centrum
        ret_sl,  USS_LB_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(13),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(14),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(15),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(16),vrep.simx_opmode_oneshot_wait) #max left




        blocksArray = env.getConcretBlockHandle(clientID)
        env.initConcretBlockPosition(clientID, blocksArray) # initialize position of the blocks

        while True: # main Control loop

            #######################################################
            # Perception Phase: Get information about environment #
            #######################################################

            # get distance to nearest block
            dist2NearestBlock, blockId, pioneer2BlockPos = env.getNearestConcretBlockDist(clientID, blocksArray, pioneerRobotHandle)
            print('dist2nearestBlock', blockId, dist2NearestBlock)
            print('Pos2nearestBlock', pioneer2BlockPos)

            # get distance to obstacle, return [distLeft, distRight]
            dist2Obstacle_LR = [getObstacleDist(USS_LF_2), getObstacleDist(USS_RF_2)]
            print('dist2obstacle', dist2Obstacle_LR)

            left4_limit = 0.5
            left1_limit = 0.5
            right1_limit = 0.5
            right4_limit = 0.5

            MID_SPEED = 2
            SLOW_SPEED = 0.4

            RUN_TIME = 2
            SHORT_TIME = 0.5


            dist_LF_4 = getObstacleDist(USS_LF_4)
            dist_LF_3 = getObstacleDist(USS_LF_3)
            dist_LF_2 = getObstacleDist(USS_LF_2)
            dist_LF_1 = getObstacleDist(USS_LF_1)
            dist_RF_1 = getObstacleDist(USS_RF_1)
            dist_RF_2 = getObstacleDist(USS_RF_2)
            dist_RF_3 = getObstacleDist(USS_RF_3)
            dist_RF_4 = getObstacleDist(USS_RF_4)



            ##############################################
            # Reasoning: figure out which action to take #
            ##############################################


            #max right or left


            #if dist_LF_4 < left4_limit | dist_RF_4 < right4_limit :

            ##ret, arr = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot_wait)
            ret, arr = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, blockId, vrep.simx_opmode_oneshot_wait)



            redBlockAngle = calculateRedBlockAngle()

            runPioneerToAngle(redBlockAngle)

            # run_motor(MOTOR_RIGHT, SLOW_SPEED, 1)

            if dist_LF_3 < left4_limit :
                run_motor(MOTOR_RIGHT, SLOW_SPEED, RUN_TIME, 1)

            elif dist_RF_3 < right4_limit:
                run_motor(MOTOR_LEFT, SLOW_SPEED, RUN_TIME, 1)

            elif dist_LF_1 < left1_limit :
                if random.random()*2 < 1:
                    run_motor(MOTOR_LEFT, SLOW_SPEED, RUN_TIME, 1)

                else :
                    run_motor(MOTOR_RIGHT, SLOW_SPEED, RUN_TIME, 1)

            else :
                run_motor(MOTOR_FRONT, MID_SPEED, 0, 1)

            # get motor speed using IF-THEN rule
            ########################                            motorSpeed = dict(speedLeft=5, speedRight=5)
            #motorSpeed = getMotorSpeed(dist2Obstacle_LR)

            ########################################
            # Action Phase: Assign speed to wheels #
            ########################################
            # assign speed to the wheels


            # collect nearest block within a short range
            env.collectNearestBlock(clientID, blocksArray, pioneerRobotHandle)

    else:
        print 'Remote API function call returned with error code: ',res
    vrep.simxFinish(clientID) # close all opened connections
else:
    print 'Failed connecting to remote API server'
    print 'Program finished'









