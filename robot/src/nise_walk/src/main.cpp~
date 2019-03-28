/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* walk node																*/
/* this node declares a walker node necesary for the robot navigation  		*/
/* This service is advertise under the topic /nise_walk						*/
/* wich request a position in x,y plane with a degree for the robot to walk	*/
/****************************************************************************/
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <string.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include "nise_walk/Walk.h"

using namespace std;
bool stop_thread=false;

class Nao_walk
{
	public:
		// ros handler
		ros::NodeHandle nh_;
		ros::ServiceServer service = nh_.advertiseService("/nise_walk", &Nao_walk::walkSrvCallBack, this);
		// publisher to nao walking
		ros::Publisher walk_pub;
		Nao_walk()
		{
			walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);
			stop_thread=false;
		}
		~Nao_walk()
		{
			stop_thread=true;
			sleep(1);
		}
	
		void walker(double x, double y, double theta)
		{
		    geometry_msgs::Pose2D goalPose;
		    goalPose.x=x;
		    goalPose.y=y;
		    goalPose.theta=theta;
		    walk_pub.publish(goalPose);
		}
		bool walkSrvCallBack(nise_walk::Walk::Request &req, nise_walk::Walk::Response &res)
		{
			ROS_INFO_STREAM("walking : "<< req);
			walker(  req.x, req.y, req.t);
			return true;
		}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "walker");
	ROS_INFO_STREAM("--- walker node initialized. ");
	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_walk ic;
	ros::spin();
	return 0;

}

