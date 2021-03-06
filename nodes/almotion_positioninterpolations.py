# -*- encoding: UTF-8 -*-

import sys
from naoqi import ALProxy
import motion

def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Example showing how to use positionInterpolations
    space        = motion.FRAME_ROBOT
    isAbsolute   = False

    # Motion of Arms with block process
    effectorList = ["LArm", "RArm"]
    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
    timeList     = [[1.0], [1.0]]         # seconds
    pathList     = [[[0.0, -0.04, 0.0, 0.0, 0.0, 0.0]],
                    [[0.0,  0.04, 0.0, 0.0, 0.0, 0.0]]]
    motionProxy.positionInterpolations(effectorList, space, pathList,
                                 axisMaskList, timeList, isAbsolute)

    # Motion of Arms and Torso with block process
    effectorList = [ "RArm"]
    axisMaskList = [motion.AXIS_MASK_VEL]
    timeList    = [[4.0]] # seconds
    dx           = 0.03                   # translation axis X (meters)
    dy           = 0.04                   # translation axis Y (meters)
    pathList     = [[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]]
    motionProxy.positionInterpolations(effectorList, space, pathList,
                                 axisMaskList, timeList, isAbsolute)


if __name__ == "__main__":
    robotIp = 'nao.local'

    if len(sys.argv) <= 1:
        print "Usage python almotion_positioninterpolations.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
