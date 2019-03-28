#!/usr/bin/env python
"""
/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* posture node																*/
/* this performs the robot non verbal comunication  						*/
/* it advertises the topic /change_posture_service wich recieves an integer	*/
/* for a required robot posture and obtional the angles to perform the pose	*/ 
/****************************************************************************/
"""
import rospy
import time
import motion
import numpy
import almath
import sys
from naoqi import ALProxy
from planner.srv import ChangePosture
from planner.srv import ChangePostureResponse
from random import *

motionProxy =0
postureProxy=0

def changePostureNAO(req):
    mode= req.desiredPosture
    # robot stands up
    if(mode == 1):
        postureProxy.goToPosture("StandInit", 0.6)
        time.sleep(3)
        motionProxy.setAngles("HeadPitch",(30*almath.TO_RAD),0.1)
        time.sleep(1)
    # robot sits
    elif(mode == 2):
        postureProxy.goToPosture("Sit", 1.0)
        time.sleep(1)
    # robot moves hear Pitch a certein angle
    elif(mode == 3):
        motionProxy.setAngles("HeadPitch",(req.angle*almath.TO_RAD),0.3)
        time.sleep(1)
    # robot moves hear yaw a certein angle
    elif(mode == 4):
        motionProxy.setAngles("HeadYaw",(req.angle*almath.TO_RAD),0.3)
        time.sleep(1)
    # robot performs the greetings pose
    elif(mode == 10):

        motionProxy.setAngles("HeadPitch",(-70*almath.TO_RAD),0.3)
        time.sleep(1)
        motionProxy.setAngles("LShoulderPitch",(-70*almath.TO_RAD),0.1)
        time.sleep(1)
        motionProxy.setAngles("LElbowYaw",(0.0*almath.TO_RAD),0.1)
        time.sleep(1)

        for x in range(0 ,5):
            if x%2:
                motionProxy.setAngles("LElbowRoll",(70*almath.TO_RAD),0.1)
            else:
                motionProxy.setAngles("LElbowRoll",(-70*almath.TO_RAD),0.1)
            time.sleep(1)
    # robot search randomly with the head
    elif(mode == 20):

        for x in range(0 ,10):
            if x%2:
                motionProxy.setAngles("HeadPitch",(uniform(-70,70)*almath.TO_RAD),0.1)
            else:
                motionProxy.setAngles("HeadYaw",(uniform(-70,70)*almath.TO_RAD),0.1)
            time.sleep(1)
        motionProxy.setAngles("HeadPitch",(-70*almath.TO_RAD),0.1)
        motionProxy.setAngles("HeadYaw",(-70*almath.TO_RAD),0.1)
        motionProxy.setAngles("HeadYaw",(70*almath.TO_RAD),0.1)
        motionProxy.setAngles("HeadYaw",(0.0*almath.TO_RAD),0.1)
    elif(mode == 30):
        motionProxy.setAngles("HeadPitch",(uniform(-70,70)*almath.TO_RAD),0.1)
        time.sleep(1)
        motionProxy.setAngles("HeadYaw",(uniform(-70,70)*almath.TO_RAD),0.1)
        time.sleep(1)
    print "Posture changed" 
    return ChangePostureResponse()

if __name__ == '__main__':
    robotIP="169.254.254.250"
    PORT=9559

    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
    rospy.init_node('change_posture_server')
    # Service declaration
    s = rospy.Service('change_posture_service', ChangePosture, changePostureNAO)
    rospy.spin()
			
		
