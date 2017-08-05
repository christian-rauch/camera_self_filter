/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * camera_self_filter.cpp
 *
 *  Created on: Oct 6, 2010
 *      Author: Christian Bersch
 */

#include <cstdio>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include <image_geometry/pinhole_camera_model.h>


#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut.h>


#include "camera_self_filter/robotMeshModel.h"



RobotMeshModel* robmod=NULL;
IplImage* ipl_maskBGRA = NULL;
IplImage* ipl_maskBW = NULL;

bool publish_mask;
bool inverted;
image_transport::Publisher mask_publisher;
image_transport::Publisher image_publisher;


void initializeGL ()
{
    glClearColor(0, 0, 0, 0);
//  glEnable(GL_LIGHTING);
//  glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                     GL_LINEAR_MIPMAP_LINEAR);
}



//function that draws the robot at current state
void display() {

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   robmod->setCamera();
   robmod->setCameraPose();
   robmod->paintRobot();
   glReadPixels(0, 0, robmod->cam_info_->width, robmod->cam_info_->height , GL_BGRA, GL_UNSIGNED_BYTE, ipl_maskBGRA->imageData);

   glutSwapBuffers();
}



void calc_and_publish_BWMask(const ros::Time time_stamp, const std::string frame_id ){
	robmod->updateRobotLinks(time_stamp);
	glutPostRedisplay();
	glutMainLoopEvent();
	cvCvtColor(ipl_maskBGRA, ipl_maskBW, CV_BGRA2GRAY);
	cvFlip(ipl_maskBW);
	if (inverted)
		cvNot(ipl_maskBW,ipl_maskBW);


   if(publish_mask){
	   std_msgs::Header header;
	   header.frame_id = frame_id;
	   header.stamp = time_stamp;
	   cv_bridge::CvImage cv_image(header,"mono8",ipl_maskBW);
	   mask_publisher.publish(cv_image.toImageMsg());
   }

}
void alpha_image_cb(const sensor_msgs::ImageConstPtr& msg_ptr){


	calc_and_publish_BWMask(msg_ptr->header.stamp, msg_ptr->header.frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg_ptr, "");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	IplImage cam_image = cv_ptr->image;
	IplImage* cam_alpha_image = cvCreateImage(cvGetSize(&cam_image), IPL_DEPTH_8U, 4);

	//b
	cvSetImageCOI(cam_alpha_image, 1);
	cvSetImageCOI(&cam_image, 1);
	cvCopy(&cam_image, cam_alpha_image);

	//g
	cvSetImageCOI(cam_alpha_image, 2);
	cvSetImageCOI(&cam_image, 2);
	cvCopy(&cam_image, cam_alpha_image);

	//r
	cvSetImageCOI(cam_alpha_image, 3);
	cvSetImageCOI(&cam_image, 3);
	cvCopy(&cam_image, cam_alpha_image);

	//alpha
	cvSetImageCOI(cam_alpha_image, 4);
	cvCopy(ipl_maskBW, cam_alpha_image);
	cvSetImageCOI(cam_alpha_image, 0);


    cv_bridge::CvImage cv_image(msg_ptr->header,"rgba8",cam_alpha_image);
    image_publisher.publish(cv_image.toImageMsg());
	cvReleaseImage(&cam_alpha_image);


}


void mask_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr){
	calc_and_publish_BWMask(msg_ptr->header.stamp, msg_ptr->header.frame_id);
}

int main(int argc, char **argv) {
	glutInit(&argc, argv);
    ros::init(argc, argv, "robot_self_filter");




    //create selfilter service
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    //set parameters
    bool publish_alpha_image;

    std::string camera_topic;
    std::string camera_info_topic;

    nh_priv.param("publish_mask",publish_mask, true);
    nh_priv.param("publish_alpha",publish_alpha_image, true);
    nh_priv.param("inverted",inverted, false);


    if(!nh_priv.getParam("camera_topic", camera_topic)) {
        ROS_ERROR("no param topic");
    }
    if(!nh_priv.getParam("camera_info_topic", camera_info_topic)) {
        ROS_ERROR("no param info");
    }
    

    ROS_INFO("camera topic %s", camera_topic.c_str());
    ROS_INFO("camera info %s", camera_info_topic.c_str());


    //create robot model
    robmod = new RobotMeshModel;

    //initialize glut
    glutInitWindowSize (robmod->cam_info_->width, robmod->cam_info_->height);
    glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);


    glutCreateWindow ("dummy");
    glutHideWindow();

    initializeGL();
    // Register glut callbacks:
    glutDisplayFunc (display);


    ipl_maskBGRA = cvCreateImage(cvSize( robmod->cam_info_->width, robmod->cam_info_->height), IPL_DEPTH_8U, 4);
    ipl_maskBW = cvCreateImage(cvSize( robmod->cam_info_->width, robmod->cam_info_->height), IPL_DEPTH_8U, 1);

    image_transport::ImageTransport it(nh);

    if (publish_mask)
    	mask_publisher = it.advertise(camera_topic + "/self_mask", 10);

    if (publish_alpha_image)
    	image_publisher = it.advertise(camera_topic + "/image_rect_color_alpha_masked", 10);

    image_transport::Subscriber image_sub;
    ros::Subscriber camerea_info_sub;
    if (publish_alpha_image){
        std::cout << "it sub" << std::endl;
    	image_sub = it.subscribe(camera_topic, 10, alpha_image_cb);
    }else{
        std::cout << "info sub: " << camera_info_topic << std::endl;
    	camerea_info_sub = nh.subscribe(camera_info_topic, 10, mask_cb);
    }



    ros::spin();


    //clean up
    cvReleaseImage(&ipl_maskBGRA);
    cvReleaseImage(&ipl_maskBW);
    delete robmod;
    return 0;
}

