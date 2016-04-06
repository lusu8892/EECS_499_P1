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

#include <transformation_generator/transformation_generator.h>


const Eigen::Matrix3d EYE_3 = Eigen::MatrixXd::Identity(3,3);
const double PI = 3.14159265359/2;
const int N = 1000; // The number of particles the system generates

using namespace std;
using namespace Eigen;

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
                const std::string& flag, const Eigen::Vector3d& rotate_axis = Eigen::Vector3d(0,0,0),
                const Eigen::Matrix4d input_trans_mat = Eigen::MatrixXd::Identity(4,4))
{
    Eigen::Affine3d random_trans_mat;
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    Eigen::Affine3d temp_trans_mat;

    temp_trans_mat.matrix() = input_trans_mat;
    
    Eigen::Vector3d input_translation = temp_trans_mat.translation();
    cout << input_translation << endl;

    cout << temp_trans_mat.linear() << endl;

    Eigen::Quaterniond input_q(temp_trans_mat.linear());
    cout << input_q.x() << " " << input_q.y() << " " << input_q.z() << " " << input_q.w() << endl; 

    Oe(0)= (*func_ptr)(a,b) + input_translation(0);
    Oe(1)= (*func_ptr)(a,b) + input_translation(1);
    Oe(2)= (*func_ptr)(a,b) + input_translation(2);
    random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins

    if (flag == "quaternion")
    {
        Eigen::Quaterniond q;
        // Eigen::Quaterniond<Scalar> 
        // double magnitude;
        // q.x() = (*func_ptr)(a,b) + input_q.x();
        // q.y() = (*func_ptr)(a,b) + input_q.y();
        // q.z() = (*func_ptr)(a,b) + input_q.z();
        // q.w() = (*func_ptr)(a,b) + input_q.w();
        Re = q.normalized().toRotationMatrix();
        // Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
        // using a quaternion, q
        random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
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

    // Eigen::Affine3d Mat_1;
    // Mat_1.linear() << Eigen::MatrixXd::Identity(3,3);
    // Mat_1.translation() << 0,0,0;

    // Eigen::Affine3d Mat_2;
    // Mat_2.linear() << Eigen::MatrixXd::Identity(3,3);
    // Mat_2.translation() << 0,0,0;

    // Eigen::Affine3d Mat_3 = Mat_1 * Mat_2;

    // // Mat_3.matrix() = Mat_1.matrix() * Mat_2.matrix();

    // T = Mat_1.translation();
    // // srand(time(NULL)); // random number seed;
    // // typedef double (*TestTypeFuncPtr)(double, double);
    // // TestTypeFuncPtr  func_uniform = getUniformRandomNum;
    // // TestTypeFuncPtr func_gaussian = getGaussianRandomNum;
    // // Mat_1 << 1,2,3,4,
    // //        4,5,6,7,
    // //        7,8,9,10,
    // //        11,12,13,14;
    // // ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("topic1", 1);
    // // //"topic1" is the name of the topic to which we will publish
    // // // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    // // std_msgs::Float64 input_float; //create a variable of type "Float64", 
    // // // as defined in: /opt/ros/indigo/share/std_msgs
    // // // any message published on a ROS topic must have a pre-defined format_1, 
    // // // so subscribers know how to interpret the serialized data transmission
   
    // ros::Rate naptime(2.0); //create a ros object from the ros “Rate” class; 
    // //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    // // input_float.data = 0.0;
    // // Eigen::Affine3d Mat_uni =  randomTransformationMatrixGenerator(getUniformRandomNum, 1, 1,"euler");
    // // Eigen::Affine3d Mat_gaussian = randomTransformationMatrixGenerator(getGaussianRandomNum, 1, 1, "quaternion");
    // double rnd;
    // // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    // // for (int i = 0; i < 10; ++i) 
    // // {
    // //     rnd = randomUniform(1, 1);
    // //     cout <<  rnd << endl;
    // //     // cout <<  T << endl;
	   // //  // naptime.sleep(); 
    // // }
    // // double x = 5;
    // // cout << *(&x) << endl;
    // // cout << Mat_uni.matrix() << endl;
    // // cout << Mat_gaussian.matrix() << endl;

    // srand(time(NULL)); // random number seed;

    // Eigen::Affine3d initial_state = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 0.04, "no");
    // initial_state.linear() = EYE_3;

    // // cout << initial_state.translation() << endl;

    // Eigen::Affine3d rot_mat_z = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 2, "arbitrary", Eigen::Vector3d(0,0,1));
    // Eigen::Vector3d rot_axis(cos(getUniformRandomNum(0, 2*PI)), sin(getUniformRandomNum(0, 2*PI)), 0);
    // Eigen::Affine3d rot_mat_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 0, 10/180, "arbitrary", rot_axis);

    // initial_state.linear() = rot_mat_a.linear() * rot_mat_z.linear() * initial_state.linear();

    // Eigen::Vector3d vec;
    // vec << 1,2,3;

    // // cout << M_1 * M_2 << endl;
    // Eigen::Quaterniond q;
    // Eigen::Quaterniond q_no;
    // q.x() = 0.1;
    // q.y() = 0.2;
    // q.z() = 0.3;
    // q.w() = 2;
    // q_no = q.normalized();

    // // MatrixBase::normalize(q);
    // cout << q_no.x() <<" " << q_no.y() << " " << q_no.z() << " "<< q_no.w() << endl;

    // double sum = q_no.x() * q_no.x() + q_no.y() * q_no.y() + q_no.z() * q_no.z() + q_no.w() * q_no.w();
    // cout << sum << endl; 
    // cout << q.x() <<" " << q.y() << " " << q.z() << " "<< q.w() << endl;

    velo_vec::velocityVector hybrid(Eigen::Vector3d(1,1,1), Eigen::Vector3d(0,0,0));

    return 0;
}