/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* hear natural language processing node									*/
/* this node declares a natural language processing service usefull  		*/
/* to identify voice commands. For that is necesary to define a dictionary 	*/
/* of known words to be recognized 											*/
/* This service is advertise under the topic /nise_question					*/
/* if the request  is one the node will reply the recognized word			*/
/****************************************************************************/
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include <string.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <std_msgs/Bool.h>

#include "nise_hear/Answer.h"

using namespace std;

class Nise_hear
{
public:
    string oldBumper;
    int counter;
    bool recognized_something;
	int answer;
    float sequencex [8];
    float sequencey [8];
    float sequencet [8];
	// ros handler
	ros::NodeHandle nh_;
	//publisher for nao speech
	ros::Publisher speech_pub;

	//publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;

	//client for starting speech recognition
	ros::ServiceClient recog_start_srv;

	//client for stoping speech recognition
	ros::ServiceClient recog_stop_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	boost::thread *spin_thread;
    int ledid;
	
	ros::ServiceServer service = nh_.advertiseService("/nise_question", &Nise_hear::answer_callback, this);
	Nise_hear()
	{
		recognized_something = false;
		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);
		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");
		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");
		recog_sub=nh_.subscribe("/word_recognized",1, &Nise_hear::speechRecognitionCB, this);
	}
	~Nise_hear()
	{

	}

	bool answer_callback(nise_hear::Answer::Request &req, nise_hear::Answer::Response &res)
	{
		ROS_INFO_STREAM("asking yes or not :)" << req.option);
		if(req.option==1)
		{
			res.answer=answer;
			return true;
		}
		std::string id;
		std::vector<string> words_;
		std_srvs::Empty emptsrv;
		recognized_something = false;
		ROS_INFO_STREAM(answer);
		counter++;
		naoqi_bridge_msgs::SetSpeechVocabularyActionGoal vocabulary;
		id= boost::lexical_cast< std::string >(counter);
		vocabulary.goal_id.id= id;

		words_.push_back("Hello");
		words_.push_back("German");
		words_.push_back("jonas");
		words_.push_back("back");
		words_.push_back("close");
		words_.push_back("open");
		words_.push_back("yes");
		words_.push_back("not");
		words_.push_back("nao");
		vocabulary.goal.words=words_;
		voc_params_pub.publish(vocabulary);
		recog_start_srv.call(emptsrv);
		ros::Duration(5).sleep();
			
		recog_stop_srv.call(emptsrv);				
		return true;
	}

	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		recognized_something = true;
		vector<string> rWord = msg->words;
		string y="yes";
		string n="not";
		if (rWord[0].compare(y) == 0)
		{
			answer=1;
		}
		else if(rWord[0].compare(n) == 0)
		{
			answer=2;
		}
		else
		{
			answer=0;
		}
		ROS_INFO_STREAM("answer " << answer);
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "NLP_node");
	ROS_INFO_STREAM("NLP_node initialized ");
	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nise_hear ic;
	ros::spin();
	return 0;

}

