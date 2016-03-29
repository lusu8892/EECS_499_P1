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
const int N = 1000; // The number of particles the system generates
const Eigen::Matrix3d EYE_3 = Eigen::MatrixXd::Identity(3,3);

using namespace std;
using namespace cv_3d;

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
// flag has two options to generate a rotation matrix: 1. quaternion, 2. rotate by arbitrary
Eigen::Affine3d randomTransformationMatrixGenerator(double (*func_ptr)(double, double), double a, double b, 
                const std::string& flag, const Eigen::Vector3d& rotate_axis = Eigen::Vector3d(0,0,0),
                const Eigen::Matrix4d input_trans_mat = Eigen::MatrixXd::Zero(4,4))
{
    Eigen::Affine3d random_trans_mat;
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    Eigen::Affine3d temp_trans_mat;

    temp_trans_mat.matrix() = input_trans_mat;
    
    Eigen::Vector3d input_translation = temp_trans_mat.translation();
    Eigen::Quaterniond input_q(temp_trans_mat.linear());

    Oe(0)= (*func_ptr)(a,b) + input_translation(0);
    Oe(1)= (*func_ptr)(a,b) + input_translation(1);
    Oe(2)= (*func_ptr)(a,b) + input_translation(2);
    random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins

    if (flag == "quaternion")
    {
        Eigen::Quaterniond q;
        // Eigen::Quaterniond<Scalar> 
        // double magnitude;
        q.x() = (*func_ptr)(a,b) + input_q.x();
        q.y() = (*func_ptr)(a,b) + input_q.y();
        q.z() = (*func_ptr)(a,b) + input_q.z();
        q.w() = (*func_ptr)(a,b) + input_q.w();
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



int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter_implemetation"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1);
	TransformationGenerator beadsGenerator;
	transformation_generator::ListOfPoints list_of_points;

	srand(time(NULL)); // random number seed;

    Eigen::Affine3d initial_state = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 0.04, "no");
    initial_state.linear() = EYE_3;

    // cout << initial_state.translation() << endl;

    Eigen::Affine3d rot_mat_z = randomTransformationMatrixGenerator(getUniformRandomNum, 0, 2, "arbitrary", Eigen::Vector3d(0,0,1));
    Eigen::Vector3d rot_axis(cos(getUniformRandomNum(0, 2*PI)), sin(getUniformRandomNum(0, 2*PI)), 0);
    Eigen::Affine3d rot_mat_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 0, 10/180, "arbitrary", rot_axis);

    // paticle filter starts from there
    initial_state.linear() = rot_mat_a.linear() * rot_mat_z.linear() * initial_state.linear();

    // convert Eigen::Affine3d transformation matrix to flat 4-by-4 Eigen::Matrix4d type
    Eigen::Matrix4d initial_state_mat = initial_state.matrix();
    // cout << initial_state.matrix() << endl;

    // make the randomly generated particles from the initial prior gaussian distribution
    // each partical is a transformation matrix from body frame to camera frame
    vector<Eigen::Affine3d> particles_set_trans_mat;
    for (int i = 0; i < N; ++i)
    {
        Eigen::Affine3d partical_trans_mat;
        partical_trans_mat = randomTransformationMatrixGenerator(getGaussianRandomNum, 0, 2, "quaternion",Eigen::Vector3d(0,0,0),initial_state_mat);
        particles_set_trans_mat.push_back(partical_trans_mat);
    }

    vector<Eigen::Affine3d> particles_set_trans_mat_update;
    vector<transformation_generator::ListOfPoints> beads_pos_update;
    
    

    double delta_time = 2;
	while(ros::ok())
	{
        particles_set_trans_mat_update.resize(particles_set_trans_mat.size());
        beads_pos_update.resize(particles_set_trans_mat.size());
        // for each partical
        for (int i = 0; i < N; ++i)
        {  
           Eigen::Affine3d particle_trans_mat_update = beadsGenerator.getNewTransformationMatrix(particles_set_trans_mat[i], delta_time);
           beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points, particle_trans_mat_update);
           beads_pos_update[i] = list_of_points;
           // particles_set_trans_mat_update.push_back(Eigen::Affine3d particle_trans_mat_update);
           // beads_pos_update.push_back();
        }
        
	}

	return 0;
}