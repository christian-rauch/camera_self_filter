/*
 * test_client.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: christian
 */



#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_self_filter/mask.h"





int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_self_filter_test");

	ros::NodeHandle nh;
	ros::ServiceClient svc = nh.serviceClient<camera_self_filter::mask>("self_mask");

	cv::namedWindow("alpha");

	while (nh.ok()){

		camera_self_filter::mask servicecall;
		servicecall.request.header.frame_id = "narrow_stereo_optical_frame";
		servicecall.request.header.stamp = ros::Time::now();
		svc.call(servicecall);

		sensor_msgs::Image mask = servicecall.response.mask_image;

		printf("width %d height %d", mask.width, mask.height);

		sensor_msgs::ImageConstPtr maskptr = boost::make_shared<sensor_msgs::Image>(boost::ref(servicecall.response.mask_image));
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(maskptr, "passthrough");
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return 0;
	    }

	    cv::imshow("alpha", cv_ptr->image);

		cvWaitKey();

	}

	return 0;
}
