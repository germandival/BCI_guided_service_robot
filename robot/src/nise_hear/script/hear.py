#!/usr/bin/env python
"""
/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* hear node																*/
/* this node records a voice message fron an extern person and plays 		*/
/* it it back this service is advertised under the topic /nise_hear 	 	*/
/* The message is recording time is given as an int in the message request	*/ 
/* if the message request is zero the node plays the recorded message 		*/
/****************************************************************************/
"""
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nise_hear.srv import Hear
motionProxy =0;

robot_IP="169.254.254.250"
robot_PORT=9559
record_path = '/home/nao/record.wav'
tts = ALProxy("ALTextToSpeech", robot_IP, robot_PORT)
audio = ALProxy("ALAudioDevice", robot_IP, robot_PORT)
record = ALProxy("ALAudioRecorder", robot_IP, robot_PORT)
aup = ALProxy("ALAudioPlayer", robot_IP, robot_PORT)

def sendPositionNAO(req):
	if req.option > 0:
		print "sendPositionNAO Script", req
		global tts, audio, record, aup, robotIP, robot_PORT, record_path
		print 'start recording...'		
		record.startMicrophonesRecording(record_path, 'wav', 16000, (0,0,1,0))
		time.sleep(req.option)
		record.stopMicrophonesRecording()
		print 'record over'
	else:
		fileID = aup.playFile(record_path, 0.7, 0)


if __name__ == '__main__':
    robotIP="169.254.254.250"
    PORT=9559
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    s = rospy.Service('/nise_hear', Hear, sendPositionNAO)
    rospy.spin()
