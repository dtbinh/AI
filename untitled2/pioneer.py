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

import vrep, math, time, env

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
        ret_sl,  ultraSonicSensorLeft = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(3),vrep.simx_opmode_oneshot_wait)
        ret_sr,  ultraSonicSensorRight = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(5),vrep.simx_opmode_oneshot_wait)

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
            dist2Obstacle_LR = [getObstacleDist(ultraSonicSensorLeft), getObstacleDist(ultraSonicSensorRight)]
            print('dist2obstacle', dist2Obstacle_LR)

            ##############################################
            # Reasoning: figure out which action to take #
            ##############################################

            # get motor speed using IF-THEN rule
            motorSpeed = dict(speedLeft=3, speedRight=4)
            #motorSpeed = getMotorSpeed(dist2Obstacle_LR)

            ########################################
            # Action Phase: Assign speed to wheels #
            ########################################
            # assign speed to the wheels
            execute(motorSpeed)

            # collect nearest block within a short range
            env.collectNearestBlock(clientID, blocksArray, pioneerRobotHandle)

    else:
        print 'Remote API function call returned with error code: ',res
    vrep.simxFinish(clientID) # close all opened connections
else:
    print 'Failed connecting to remote API server'
    print 'Program finished'
