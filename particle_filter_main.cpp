// paticle_filter_main.cpp //
/// This is the main function of the implementation of particle filter to track
/// the beads position based on the segmented image source

#include <ros/ros.h>
#include <transformation_generator/transformation_generator.h>
#include <particle_weight/particle_weight.h>
#include <cwru_opencv_common/opencv_geometry_3d.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

double randomUniform(double dMinValue, double dMaxValue)
{
    double pRandomValue = (double)(rand()/(double)RAND_MAX);
    pRandomValue = pRandomValue*(dMaxValue-dMinValue)+dMinValue;
    return pRandomValue;
}

// A function randomly generate transformation matrix
Eigen::Affine3d randomTransformationMatrixGenerator()
{
    Eigen::Affine3d random_trans_mat;
    Eigen::Vector3d Oe;
    Oe(0)= gaussianNumberGenerator(-0.01, 0.01);
    Oe(1)= gaussianNumberGenerator(-0.01, 0.01);
    Oe(2)= gaussianNumberGenerator(-0.01, 0.01);
    random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins
    Eigen::Quaterniond q;
    // Eigen::Quaterniond<Scalar> 
    // double magnitude;
    q.x() = gaussianNumberGenerator(-1, 1);
    q.y() = gaussianNumberGenerator(-1, 1);
    q.z() = gaussianNumberGenerator(-1, 1);
    q.w() = gaussianNumberGenerator(-1, 1);
    Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
    // using a quaternion, q
    random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
    return random_trans_mat;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter_implemetation"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1);
	TransformationGenerator beadsGenerator;
	transformation_generator::ListOfPoints list_of_points;

	srand(time(NULL)); // random number seed;

	eye3 << Eigen::MatrixXd::Identity(3,3);
	Eigen::affine3d initial_state;


	while(ros::ok())
	{
		beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points);
		beads_pos_pub.publish(list_of_points);
		ros::Duration(1.0).sleep();
	}

	return 0;
}