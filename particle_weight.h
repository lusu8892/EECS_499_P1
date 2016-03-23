#ifndef PARTICLE_WEIGHT_H_
#define PARTICLE_WEIGHT_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_opencv_common/opencv_geometry_3d.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <transformation_generator/ListOfPoints.h>


using namespace std;

class ParticleWeight
{
public:
	ParticleWeight(ros::NodeHandle* nodehandle);
	void drawBeads();



private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
	// some objects to support subscriber, service, and publisher
	ros::Publisher particle_weight_pub_;
	ros::Subscriber img_sub_


	// member methods as well:
	void initializePublishers();
	void initializeSubscribers();

	void imageCallback(const sensor_msgs::ImageConstPtr& segmented_image);
};

#endif