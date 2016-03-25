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
Eigen::Affine3d randomTransformationMatrixGenerator(double (*func_ptr)(double, double), double a, double b, const std::string& flag, Eigen::Vector3d rotate_axis)
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
    // typedef double (*TestTypeFuncPtr)(double, double);
    // TestTypeFuncPtr  func_uniform = getUniformRandomNum;
    // TestTypeFuncPtr func_gaussian = getGaussianRandomNum;
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
    // Eigen::Affine3d Mat_uni =  randomTransformationMatrixGenerator(getUniformRandomNum, 1, 1,"euler");
    // Eigen::Affine3d Mat_gaussian = randomTransformationMatrixGenerator(getGaussianRandomNum, 1, 1, "quaternion");
    double rnd;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    // for (int i = 0; i < 10; ++i) 
    // {
    //     rnd = randomUniform(1, 1);
    //     cout <<  rnd << endl;
    //     // cout <<  T << endl;
	   //  // naptime.sleep(); 
    // }
    // double x = 5;
    // cout << *(&x) << endl;
    // cout << Mat_uni.matrix() << endl;
    // cout << Mat_gaussian.matrix() << endl;

    Eigen::Vector3d arbitary_axis(1,2,3);
    // cout << arbitary_axis.normalize() << endl;

    // Eigen::Affine3d initial_state = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 0.04, "no");
    // initial_state.linear() << Eigen::MatrixXd::Identity(3,3);
    Eigen::Affine3d rot_z = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 2, "euler", (0,0,1));
    // Eigen::Vector3d rot_a(cos(getUniformRandomNum(0, 2*M_PI)), sin(getUniformRandomNum(0, 2*M_PI)), 0);
    Eigen::Affine3d rot_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 0, 10/180, "euler", (0,0,1));

    return 0;
}