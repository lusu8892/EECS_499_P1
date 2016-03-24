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
#include <geometry_msgs/Point.h>
#include <transformation_generator/ListOfPoints.h>

// define a namespace velo_vec to avoid name confliction
namespace velo_vec
{
    // define a structure velocityVector used to contain angular and translational velocity
    struct velocityVector 
    {
    	Eigen::Vector3d transV; // translational velocity part
        Eigen::Vector3d angV; // angular velocity part
        // constructor
        velocityVector(Eigen::Vector3d transV_ = Eigen::Vector3d(1, 2, 3), Eigen::Vector3d angV_ = Eigen::Vector3d(1, 2, 3)):
        transV(transV_)
        angV(angV_)
        {
        }
    };
  
}

// define a class, including a constructor, member variables and member functions

class TransformationGenerator
{
public:
    TransformationGenerator();
    // TransformationGenerator(ros::NodeHandle* nodehandle);
    // given randomly generated transformation matrix, this function can give a list of
    // beads position
    void getBeadsPosition(int beads_number, int row_num, int col_num, transformation_generator::ListOfPoints& list_of_points);

    // // A function randomly generate transformation matrix
    // Eigen::Affine3d randomTransformationMatrixGenerator();

private:
    // put private member data here; "private" data will only be available to member functions of this class;
    // ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    // ros::Publisher beads_pos_pub_;

    // member methods as well:
    // void initializePublishers();

    // A function randomly generate transformation matrix
    Eigen::Affine3d randomTransformationMatrixGenerator();

    // A function generate random number
    double gaussianNumberGenerator(double LO, double HI);

    // A funtion generate a Exponential Matrix based on random body velocity
    Eigen::Affine3d getExpoMatrix(velo_vec::velocityVector rand_body_velo, double delta_time, Eigen::Affine3d trans_mat);

    // A function to convert vector to skew-symmetric matrix
    Eigen::Matrix3d TransformationGenerator::getSkewSymMatrix(Eigen::Vector3d vector_in)

};

#endif