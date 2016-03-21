// transformation_generator.h head file //
/// Su Lu: Mar, 2016
/// Include this file in "transformation_generator.cpp", and in any main that uses this library.
/// This class has funtion to generate beads position depended on the numbers and given a
/// transformation matrix

#ifndef TRANSFORMATION_GENERATOR_H_
#define TRANSFORMATION_GENERATOR_H_

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <eigen_msg.h>
#include <geometry_msgs/Polygon.h>

using namespace std;
// define a class, including a constructor, member variables and member functions

class TransformationGenerator
{
public:
	TransformationGenerator(ros::NodeHandle& nodehandle);

	// given randomly generated transformation matrix, this function can give a list of
	// beads position
	void getBeadsPosition(int beads_number, int row_num, int col_num, vector<Eigen::Vector3d>& beads_position);
	
	// convert a vector of Eigen::Vector3d points into ros geometry/Points message

private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
	// some objects to support subscriber, service, and publisher
	ros::Publisher beads_pos_pub_;

	// member methods as well:
	void initializePublishers();

	// A function randomly generate transformation matrix
	Eigen::Affine3d randomTransformationMatrixGenerator();

};

#endif