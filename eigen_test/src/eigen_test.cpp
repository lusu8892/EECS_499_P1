#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>


double randomUniform(double dMinValue, double dMaxValue)
{
    double pRandomValue = (double)(rand()/(double)RAND_MAX);
    pRandomValue = pRandomValue*(dMaxValue-dMinValue)+dMinValue;
    return pRandomValue;
}

using namespace std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "eigen_test"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    Eigen::Matrix3d I;
    Eigen::Vector3d T;

    Eigen::Affine3d Mat_1;
    Mat_1.linear() << Eigen::MatrixXd::Identity(3,3);
    Mat_1.translation() << 0,0,0;

    Eigen::Affine3d Mat_2;
    Mat_2.linear() << Eigen::MatrixXd::Identity(3,3);
    Mat_2.translation() << 0,0,0;

    Eigen::Affine3d Mat_3 = Mat_1 * Mat_2;

    // Mat_3.matrix() = Mat_1.matrix() * Mat_2.matrix();

    T = Mat_1.translation();
    // srand(time(NULL)); // random number seed;

    // Mat_1 << 1,2,3,4,
    //        4,5,6,7,
    //        7,8,9,10,
    //        11,12,13,14;
    // ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("topic1", 1);
    // //"topic1" is the name of the topic to which we will publish
    // // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    // std_msgs::Float64 input_float; //create a variable of type "Float64", 
    // // as defined in: /opt/ros/indigo/share/std_msgs
    // // any message published on a ROS topic must have a pre-defined format_1, 
    // // so subscribers know how to interpret the serialized data transmission
   
    ros::Rate naptime(2.0); //create a ros object from the ros “Rate” class; 
    //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    // input_float.data = 0.0;
    
    double rnd;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    for (int i = 0; i < 10; ++i) 
    {
        rnd = randomUniform(-4.0, 4.0);
        cout <<  rnd << endl;
        // cout <<  T << endl;
	    // naptime.sleep(); 
    }

    return 0;
}