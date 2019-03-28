/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* planner node																*/
/* this node coordinates the execution of the BCI guided service robot 		*/
/* it interprets the robot sensory information to execute the patient	 	*/
/* commands accoording with the patient's needs.							*/
/* This node interacts with the BCI system through five menus each one 		*/
/* with a maximum number of 9 options										*/
/* menu\options	1    	2  		3		4		5		6	7	8	9		*/
/* menu1:		good	bad		pain	joke	alone						*/
/* menu2		talk	walk	joke	bring	leave						*/
/* menu3		doc		german	stefan	pablo	mom							*/
/* menu4		yes		no													*/
/* menu5		5min	10min	15min	30min	1h		3h	6h	24	never	*/
/****************************************************************************/
// c++ realated libraries
#include <iostream>
#include <string.h>
#include <ctime>
// ros related libraries
#include <ros/ros.h>
// implemented ros node related libraries 
#include "nise_talk/Talk.h"
#include "nise_walk/Walk.h"
#include "nise_interface/Interface.h"
#include "nise_face_detection/NaoFace.h"
#include "nise_hear/Hear.h"
#include "nise_hear/Answer.h"
#include "nise_pose/ChangePosture.h"

using namespace std;
using namespace ros;

int counter=0;
vector <string> message;

class Nao_planner
{
	public:
	// ros crients and publishers
	ros::NodeHandle nh_;
	ros::Publisher speech_pub;
	ros::ServiceClient talker_client;
	ros::ServiceClient walker_client;
	ros::ServiceClient interface_client;
	ros::ServiceClient face_recognition_client;
	ros::ServiceClient hear_client;
	ros::ServiceClient answer_client;
	ros::ServiceClient pose_client;
	// atributes to call the services
	nise_talk::Talk talk_srv;
	nise_walk::Walk walk_srv;
	nise_interface::Interface interface_srv;
	nise_face_detection::NaoFace face_recognition_srv;
	nise_hear::Hear hear_srv;
	nise_hear::Answer answer_srv;
	nise_pose::ChangePosture pose_srv;
	// planner attributes 
	int ans;
	int bci_menu;
	bool end_of_demo;
	long waiting_time;
	long eod_time;
	long current_time;
	
	Nao_planner()
	{
		// initialization of clients and publishers
		talker_client = nh_.serviceClient<nise_talk::Talk>("/nise_talk");
		walker_client = nh_.serviceClient<nise_walk::Walk>("/nise_walk");
		interface_client = nh_.serviceClient<nise_interface::Interface>("/interface_request");
		face_recognition_client = nh_.serviceClient<nise_face_detection::NaoFace>("/recognize_face");
		hear_client = nh_.serviceClient<nise_hear::Hear>("/nise_hear");
		answer_client = nh_.serviceClient<nise_hear::Answer>("/nise_question");
		pose_client = nh_.serviceClient<nise_pose::ChangePosture>("/change_posture_service");
		ROS_INFO_STREAM("--- planner constructor initialized.");
		// beggin of interaction with the patient
		// greeting posture
		pose_srv.request.desiredPosture = 1;
		pose_client.call(pose_srv);
		pose_srv.request.desiredPosture = 10;
		pose_client.call(pose_srv);
		//verbal communication
		talk_srv.request.message.push_back(" hi jonas. i am nao your personal assistant.");
		ros::Duration(5).sleep();
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		// first menu initialization
		bci_menu = 1;
		end_of_demo = false;
		pose_srv.request.desiredPosture = 3;
		pose_srv.request.angle = 0.0;
		// video strreamin initialization
		interface_srv.request.option=42;
		interface_client.call(interface_srv);
		// set initial posture
		pose_srv.request.desiredPosture = 1;
		pose_client.call(pose_srv);
		pose_srv.request.desiredPosture = 3;
		pose_srv.request.angle = -30.0;
		pose_client.call(pose_srv);
		for(;;)
		{
			// identify if one of the subroutines was accomplished
			if (end_of_demo)
			{
				// menu 5 indicates gets the time for a new system execution
				bci_menu = 5;
				// end of one demo execution
				end_of_demo = false;
				// end of demo time
				eod_time = get_minute();
				// verbal feedback to the patient
				talk_srv.request.message.push_back(" my work here is done. see you soon");
				talker_client.call(talk_srv);
				talk_srv.request.message.clear();
			}
			else
			{
				// wait for the time to engage interaction
				current_time = get_minute();
				if(current_time-eod_time < waiting_time)
					continue;
			}
			// select the next menu
			switch(bci_menu)
			{
				waiting_time=0;
                case 1:
					menu_1();
					break;
				case 2:
					menu_2();
					break;
				case 3:
					menu_3();
					break;
				case 4:
					menu_4();
					break;
			}
		}
		
	}
	long get_minute()
	{
		// get the current time point
		const std::time_t now = std::time(nullptr); 
		// convert it to (local) calendar time
		const std::tm calendar_time = *std::localtime( std::addressof(now));
		//actual time in minutes
		long time_in_minutes= calendar_time.tm_hour*60 + calendar_time.tm_min;
		return time_in_minutes;
	}
	void menu_1()
	{
		//        	1    	2  		3		4		5
		//menu1:	good	bad		pain	joke	alone
		// verbal feedback to the patient
		talk_srv.request.message.push_back(" how are you feeling?");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ans=bci_interface(bci_menu);
		switch(ans)
		{
                case 1:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you are feeling good. can i do something for you");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 2;
					break;
				case 2:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you are feeling bad. can i do something for you");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 2;
					break;
				case 3:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you are feeling pain. maybe i should bring someone");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 3;
					break;
				case 4:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will tell you a joke");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					talk_srv.request.message.push_back(" what  is  an  ego?.        it  is  the  little  argentinian  we  all  have.    hi    francisco");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					end_of_demo = true;
					break;
				case 5:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you are feeling alone. can i do something for you");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 2;
					break;
				case 6:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you want me to leave you alone");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					end_of_demo = true;
					break;
				default:
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i dont understand this command can you repeat please?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					break;

		}
	}
	
	void menu_2()
	{
		//        	1    	2  		3		4		5
		//menu2		talk	walk	joke	bring	leave me
		talk_srv.request.message.push_back(" what do you want to do?");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ans=bci_interface(bci_menu);
		switch(ans)
		{
                case 1:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you want to talk");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					talk();
					end_of_demo = true;
					break;
				case 2:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you want to walk");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					walk();
					end_of_demo = true;
					break;
				case 3:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back("  i understand you are feeling bored. i will tell you a joke");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					talk_srv.request.message.push_back(" what  is  an  ego?.        it  is  the  little  argentinian  we  all  have.    hi    francisco");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					end_of_demo = true;
					break;
				case 4:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" who should i bring?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 3;
					break;
				case 5:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i understand you want me to leave you alone");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					end_of_demo = true;
					break;
				default:
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i dont understand this command can you repeat please?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 2;
					break;

		}
	}

	void menu_3()
	{
		//        	1    	2  		3		4		5
		//menu4		doc		german	stefan	pablo	mom
		ans=bci_interface(bci_menu);
		switch(ans)
		{
                case 1:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will bring you a doctor");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bring(ans);
					end_of_demo = true;
					break;
				case 2:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will bring you german");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bring(ans);
					end_of_demo = true;
					break;
				case 3:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will bring you stefan");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bring(ans);
					end_of_demo = true;
					break;
				case 4:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will bring you pablo");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bring(ans);
					end_of_demo = true;
					break;
				case 5:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will bring you mom");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bring(ans);
					end_of_demo = true;
					break;
				default:
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i dont understand this command can you repeat please?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 3;
					break;
		}
	}
	void menu_4()
	{
		//        	1    	2  		3		4		5
		//menu4:	yes		not
		// verbal feedback to the patient
		talk_srv.request.message.push_back(" how are you feeling?");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ans=bci_interface(bci_menu);
		switch(ans)
		{
                case 1:	
					talk_srv.request.message.push_back(" Who should i bring?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 3;
					break;
				default:
					talk_srv.request.message.push_back(" I will stay here in case you need something");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					end_of_demo = true;
					break;
		}
	}
	void menu_5()
	{
		//        	1    	2  		3		4		5	6	7	8	9
		//menu5		5min	10min	15min	30min	1h	3h	6h	24	never
		ans=bci_interface(bci_menu);
		switch(ans)
		{
                case 1:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 5;
					break;
				case 2:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 10;
					break;
				case 3:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 15;
					break;
				case 4:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feelingo");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 30;
					break;
				case 5:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 60;
					break;
				case 6:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 180;
					break;
				case 7:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 360;
					break;
				case 8:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feelingo");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 1440;
					break;
				case 9:	
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i will come back in five minutes to see how are you feeling");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					bci_menu = 1;
					waiting_time = 42;
					break;
				default:
					// verbal feedback to the patient
					talk_srv.request.message.push_back(" i dont understand this command can you repeat please?");
					talker_client.call(talk_srv);					
					break;
		}
	}
	
	int bci_interface(int menu)
	{
		// get the response from the bci interface  and break queh the interface sends a zero
		interface_srv.request.option=menu;
		for(int i=0;i<50;i++)
		{
			interface_client.call(interface_srv);
			int answer = interface_srv.response.answer;
			if (answer > 0)
				break;
			ros::Duration(5).sleep();
		}
		ros::Duration(5).sleep();
		
		return interface_srv.response.answer;
	}
	void walk()
	{
		//routine for a walk while video is straming through the interface node
		float auxsequencex [8]= {0.5,0,    0.5, 0,   0.5 , 0,  0.5, 0};
		float auxsequencey [8]= {0 , 0,    0,   0,   0 , 0,    0,   0};
		float auxsequencet [8]= {0 ,-1.57, 0,   -1.57,0 ,-1.57, 0,   -1.57};
		for(int j=0; j<8;j++)
		{
            walk_srv.request.x = auxsequencex[j];
            walk_srv.request.y = auxsequencey[j];
            walk_srv.request.t = auxsequencet[j];
			walker_client.call(walk_srv);
			ros::Duration(10).sleep();
        }			
	}
	void bring(int option)
	{
		// routine to bring an external person
		float auxsequencex [8]= {0,    	0.5, 	0,   	0.5,	0,		0,		0,		0};
		float auxsequencey [8]= {0,    	0,   	0,   	0,		0,		0,		0,		0 };
		float auxsequencet [8]= {-3, 	0,   	1.57,	0,		0.2,	-0.5,	0.5,	-0.2 };
		for(int j=0; j<8;j++)
		{
            walk_srv.request.x = auxsequencex[j];
            walk_srv.request.y = auxsequencey[j];
            walk_srv.request.t = auxsequencet[j];
			walker_client.call(walk_srv);
			pose_srv.request.desiredPosture = 30;
			pose_client.call(pose_srv);
			ros::Duration(10).sleep();
        }
		pose_srv.request.desiredPosture = 20;
		pose_client.call(pose_srv);
		for(int i=0;i<20;i++)
		{
			face_recognition_srv.request.name="german";
			face_recognition_client.call(face_recognition_srv);
			ROS_INFO_STREAM("I am watching at "<<face_recognition_srv.response.personInImage);
			if(face_recognition_srv.response.personInImage=="German")
			{	
				talk_srv.request.message.push_back("I am watching at german");
				talker_client.call(talk_srv);
				talk_srv.request.message.clear();
				break;
			}
			ros::Duration(5).sleep();
		}
		// greeting routine
		pose_srv.request.desiredPosture = 10;
		pose_client.call(pose_srv);
		ros::Duration(10).sleep();
		pose_srv.request.desiredPosture = 1;
		pose_client.call(pose_srv);
		switch(option)
		{
                case 1:	
					// verbal feedback to the target person
					talk_srv.request.message.push_back(" hi doctor. can you come with me to visit jonas?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					break;
				case 2:	
					// verbal feedback to the target person
					talk_srv.request.message.push_back(" hi german. can you come with me to visit jonas?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					break;
				case 3:	
					// verbal feedback to the target person
					talk_srv.request.message.push_back(" hi stefan. can you come with me to visit jonas?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					break;
				case 4:	
					// verbal feedback to the target person
					talk_srv.request.message.push_back(" hola pablo. can you come with me to visit jonas?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					break;
				case 5:	
					// verbal feedback to the target person
					talk_srv.request.message.push_back(" hi mom of jonas. can you come with me to visit jonas?");
					talker_client.call(talk_srv);
					talk_srv.request.message.clear();
					break;
		}
		ros::Duration(10).sleep();
		answer_srv.request.option = 0;
		answer_client.call(answer_srv);
		answer_srv.request.option = 1;
		answer_client.call(answer_srv);
		int person_answer=answer_srv.response.answer;
		ROS_INFO_STREAM("The person answered " << person_answer);
		if(person_answer==1)
		{
			// verbal feedback to the target person
			talk_srv.request.message.push_back(" please follow me to visit jonas ");
			talker_client.call(talk_srv);
			talk_srv.request.message.clear();
			pose_srv.request.desiredPosture = 3;
			pose_srv.request.angle = -30.0;
			pose_client.call(pose_srv);
			// comeback to the patient
			float auxsequencex_back [4]= {0,    	0.5, 	0,   	0.5};
			float auxsequencey_back [4]= {0,    	0,   	0,   	0 };
			float auxsequencet_back [4]= {-3, 		0,   	-1.57,	0 };
			for(int j=0; j<4;j++)
			{
		        walk_srv.request.x = auxsequencex_back[j];
		        walk_srv.request.y = auxsequencey_back[j];
		        walk_srv.request.t = auxsequencet_back[j];
				walker_client.call(walk_srv);
				ros::Duration(10).sleep();
		    }
			// verbal feedback to the paient
			talk_srv.request.message.push_back(" hi jonas look who is here to visit you.");
			talker_client.call(talk_srv);
			talk_srv.request.message.clear();
		}
		else
		{
			// verbal feedback to the target person
			talk_srv.request.message.push_back(" i understandyou can not come to visit jonas right now please record a message for him ");
			talker_client.call(talk_srv);
			talk_srv.request.message.clear();
			talk_srv.request.message.push_back(" i will record a message the next twuenty seconds ");
			
			talker_client.call(talk_srv);
			talk_srv.request.message.clear();
			ros::Duration(10).sleep();
			hear_srv.request.option = 20;
			hear_client.call(hear_srv);
			// verbal feedback to the target person
			talk_srv.request.message.push_back(" i will give your message to jonas ");
			talker_client.call(talk_srv);
			talk_srv.request.message.clear();
			// comeback to the patient
			float auxsequencex_back [4]= {0,    	0.5, 	0,   	0.5};
			float auxsequencey_back [4]= {0,    	0,   	0,   	0 };
			float auxsequencet_back [4]= {-3, 		0,   	-1.57,	0 };
			for(int j=0; j<4;j++)
			{
		        walk_srv.request.x = auxsequencex_back[j];
		        walk_srv.request.y = auxsequencey_back[j];
		        walk_srv.request.t = auxsequencet_back[j];
				walker_client.call(walk_srv);
				ros::Duration(10).sleep();
		    }
			hear_srv.request.option = 0;
			hear_client.call(hear_srv);
			ros::Duration(20).sleep();
		}
			
	}
	void talk()
	{
		// talking soubroutine
		talk_srv.request.message.push_back(" I understand that you are feeling lonely and want to talk");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ros::Duration(5).sleep();
		talk_srv.request.message.push_back(" it is good that i am here to talk with you");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ros::Duration(5).sleep();
		talk_srv.request.message.push_back(" today the weather is really good. loook how beutifull is outside");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ros::Duration(5).sleep();
		talk_srv.request.message.push_back(" do you want me to call someone to open the window so you can enjoy the weater?");
		talker_client.call(talk_srv);
		talk_srv.request.message.clear();
		ros::Duration(5).sleep();
		bci_menu = 4;
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	ROS_INFO_STREAM("--- planner node initialized.");
	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_planner planner;
	ros::spin();
	return 0;

}

