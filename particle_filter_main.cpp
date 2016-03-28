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

const double PI = 3.14159265359/2;

double getUniformRandomNum(double dMinValue, double dMaxValue)
{
    double pRandomValue = (double)(rand()/(double)RAND_MAX);
    pRandomValue = pRandomValue*(dMaxValue-dMinValue)+dMinValue;
    return pRandomValue;
}

double getGaussianRandomNum(double mean, double std_deviation)
{
    static double V1, V2, S;
    static int phase = 0;
    double X;
     
    if ( phase == 0 ) {
        do {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
             
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
         
        X = V1 * sqrt(-2 * log(S) / S);
    } else
        X = V2 * sqrt(-2 * log(S) / S);
         
    phase = 1 - phase;
 
    X = X * std_deviation + mean;
    return X;
}

// A function randomly generate transformation matrix
// the first input parameter type is function pointer which require you to insert what function name you 
// pick, and the last two parameters are input parameters for the function you pick.
// flag has two options to generate a rotation matrix: 1. quaternion, 2.euler(XYZ). 3. rotate by arbitrary
Eigen::Affine3d randomTransformationMatrixGenerator(double (*func_ptr)(double, double), double a, double b, 
                const std::string& flag, const Eigen::Vector3d& rotate_axis)
{
    Eigen::Affine3d random_trans_mat;
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;

    Oe(0)= (*func_ptr)(a,b);
    Oe(1)= (*func_ptr)(a,b);
    Oe(2)= (*func_ptr)(a,b);
    random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins

    if (flag == "quaternion")
    {
        Eigen::Quaterniond q;
        // Eigen::Quaterniond<Scalar> 
        // double magnitude;
        q.x() = (*func_ptr)(a,b);
        q.y() = (*func_ptr)(a,b);
        q.z() = (*func_ptr)(a,b);
        q.w() = (*func_ptr)(a,b);
        Re = q.normalized().toRotationMatrix();
        // Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
        // using a quaternion, q
        random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
    }
    else if (flag == "euler")
    {
        Re = Eigen::AngleAxisd((*func_ptr)(a,b)*M_PI, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd((*func_ptr)(a,b)*M_PI,  Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd((*func_ptr)(a,b)*M_PI, Eigen::Vector3d::UnitZ());
    }
    else if (flag == "arbitrary")
    {
        Re = Eigen::AngleAxisd((*func_ptr)(a,b)*M_PI, rotate_axis.normalized());
    }
    else
    {

    }

    return random_trans_mat;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter_implemetation"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1);
	TransformationGenerator beadsGenerator;
	transformation_generator::ListOfPoints list_of_points;

	srand(time(NULL)); // random number seed;

	Eigen::affine3d initial_state = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 0.04, "no");
    initial_state.linear() << Eigen::MatrixXd::Identity(3,3);
    Eigen::affined rot_z = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 2, "euler", (0,0,1));
    Eigen::Vector3d rot_a(cos(getUniformRandomNum(0, 2*M_PI)), sin(getUniformRandomNum(0, 2*M_PI)), 0);
    Eigen::affined rot_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 0, 10/180, "euler", (0,0,1));

	while(ros::ok())
	{
        // beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points);
        beads_pos_pub.publish(list_of_points);
        ros::Duration(1.0).sleep();
	}

	return 0;
}