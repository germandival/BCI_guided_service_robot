/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* talk node																*/
/* this node declares talk node necesary for verbal communication   		*/
/* This service is advertise under the topic /nise_talk						*/
/* wich recieves an array of strings with the words for the robot to say	*/
/****************************************************************************/
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include "nise_talk/Talk.h"

using namespace std;
using namespace ros;
int counter=0;
vector <string> message;

class Nao_talk
{
	public:
	ros::NodeHandle nh_;
	ros::Publisher speech_pub;
	ros::ServiceServer service = nh_.advertiseService("/nise_talk", &Nao_talk::talk, this);
	Nao_talk()
	{
		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
	}

	bool talk( nise_talk::Talk::Request &req, nise_talk::Talk::Response &res)
	{
		naoqi_bridge_msgs::SpeechWithFeedbackActionGoal text;
		vector <string> textNao;
		textNao.push_back(req.message[0]);
		for(int i=0;i<textNao.size();i++)
		{
		   //Increase unique id
		   counter++; 
		   text.goal_id.id=counter;
		   text.goal.say=textNao[i];
		   //publis nao text to speak
		   speech_pub.publish(text);
		   //Pause between words
		   ros::Duration(1).sleep(); 
		 }

		return true;
	}
};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
	ROS_INFO_STREAM("--- talker node initialized.");
	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_talk talker;
	ros::spin();
	return 0;

}

