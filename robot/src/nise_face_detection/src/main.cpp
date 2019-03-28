/****************************************************************************/
/*                                                                        	*/
/* Date: 3.2.2019                                                       	*/
/* Author: German Diez Valencia    german.diez@tum.de			           	*/
/* Neuro ispired systems engineering                                       	*/
/* face detection node														*/
/* this node face detection node that later on connects to a face 			*/
/* recognition node in order to identify faces in the robot camera image 	*/
/* This service is advertise under the topic /recognize_face				*/
/* if a known person is in the camera image the service returns the person	*/
/* name	otherwise it returns the string "no_one"							*/
/****************************************************************************/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "nise_face_detection/RoiFace.h"
#include "nise_face_detection/NaoFace.h"
#include <string.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <image_transport/image_transport.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include <string.h>
#include <math.h>

using namespace std;
using namespace aruco;
using namespace cv;

/** Global variables */
String face_cascade_name = "/home/nise/nise/src/nise_face_detection/haarcascade_frontalface_alt.xml";

CascadeClassifier face_cascade;
vector<Rect> ROIS;
String nameSave="None";
String DB_path="/home/nise/nise/src/nise_face_detection/faceDB/"+nameSave+"/";
int saveIndex=0;
class Nao_face_detection
{
public:
    // ros handler
    ros::NodeHandle nh_;

    ros::ServiceClient client;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::ServiceServer service = nh_.advertiseService("/recognize_face", &Nao_face_detection::faceSrvCallBack, this);
    Mat image;
    Nao_face_detection():it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_robot/naoqi_driver/camera/front/image_raw", 1, &Nao_face_detection::imageCb, this);
        client = nh_.serviceClient<nise_face_detection::RoiFace>("face_service");
    }
    ~Nao_face_detection()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& img_raw)
    {
        /* Create the bridge between ros and opencv format*/
        cv_bridge::CvImagePtr  cv_img_ptr;
        /*Convert from ROS image to OpenCv image format*/
        Mat gray_img;
        try{
           cv_img_ptr = cv_bridge::toCvCopy(img_raw,"bgr8");
           cv_img_ptr->image.copyTo(image);

        }
        catch (cv_bridge::Exception& e){
           ROS_ERROR("cv_bridge exception from camera: %s", e.what());
           return;
        }

        if( !face_cascade.load( face_cascade_name ) ){
            ROS_ERROR("********Error loading face");
        }
		cv::cvtColor(image,gray_img, CV_BGR2GRAY);
		face_cascade.detectMultiScale( gray_img, ROIS, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) );
		// to create the face database
		for( size_t i = 0; i < ROIS.size(); i++ )
		{
			Rect roi=ROIS[i];
			cv::Mat crop = image(roi);
			cv::imshow("crop", crop);
			string savePath=DB_path+std::to_string(saveIndex)+".png";
			imwrite( savePath, crop );
			saveIndex++;
		}
		cv::imshow("faces", image);
		cv::waitKey(1);
        if(nameSave!="None")
        {
           cv::cvtColor(image,gray_img, CV_BGR2GRAY);
           face_cascade.detectMultiScale( gray_img, ROIS, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) );
           for( size_t i = 0; i < ROIS.size(); i++ )
           {
                Rect roi=ROIS[i];
                cv::Mat crop = image(roi);
                cv::imshow("crop", crop);
                string savePath=DB_path+std::to_string(saveIndex)+".png";
                imwrite( savePath, crop );
                saveIndex++;
           }
           cv::imshow("faces XD", image);
           cv::waitKey(1);
        }
    }

    bool faceSrvCallBack(nise_face_detection::NaoFace::Request &req, nise_face_detection::NaoFace::Response &res)
    {
        string person=req.name;

        ROS_INFO_STREAM(person);
        Mat gray_img;
        cv::cvtColor(image,gray_img, CV_BGR2GRAY);
        face_cascade.detectMultiScale( gray_img, ROIS, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(50, 50) );
        nise_face_detection::RoiFace srv;
        for( size_t i = 0; i < ROIS.size(); i++ )
        {
             Rect roi=ROIS[i];
             cv::Mat crop = image(roi);
             cv::imshow("crop", crop);
             srv.request.x.push_back(roi.x);
             srv.request.y.push_back(roi.y);
             srv.request.h.push_back(roi.height);
             srv.request.w.push_back(roi.width);
        }
        if(ROIS.size()>0)
        {
            ROS_INFO_STREAM(srv.request);
            ROS_INFO_STREAM(client.call(srv)); /*Call to the service*/
            client.call(srv);
            ROS_INFO_STREAM("I was asked for: "<<person);
            for (std::vector<string>::const_iterator i = srv.response.name.begin(); i != srv.response.name.end(); ++i)
            {
                string aux=*i;
                ROS_INFO_STREAM("I recognized: "<<aux);
                res.personInImage=aux;
                return true;
            }
        }
        res.inImage=false;
        res.personInImage="No_one";
        cv::imshow("faces ", image);
        cv::waitKey(1);
        return true;
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_recognition_node");
    Nao_face_detection ic;
    //ic.sendManual();
    ros::spin();
    return 0;

}
