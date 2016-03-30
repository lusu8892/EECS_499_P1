// paticle_filter_main.cpp //
/// This is the main function of the implementation of particle filter to track
/// the beads position based on the segmented image source
/// the particle are presented as transformation matrix from body to camera frame
/// based on each particle, 9 beads were created by opencv function

#include <ros/ros.h>
#include <transformation_generator/transformation_generator.h>
// #include <particle_weight/particle_weight.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cwru_opencv_common/opencv_geometry_3d.h>
#include <cwru_opencv_common/projective_geometry.h>

const double PI = 3.14159265359;
const int N = 1000; // The number of particles the system generates
const Eigen::Matrix3d EYE_3 = Eigen::MatrixXd::Identity(3,3);
const double RADIUS = 0.0082/2;

using namespace std;
using namespace cv_3d;

cv::Mat g_frame_in;
bool g_new_image;

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
        Re = Eigen::AngleAxisd((*func_ptr)(a,b)*PI, rotate_axis.normalized());
    }
    else
    {

    }

    return random_trans_mat;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& segemented_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvShare(segemented_image, "mono8");
        g_frame_in = cv_ptr->image.clone();
        g_new_image = true;
    }
    catch (cv_bridge::Exception& e)
    {   
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter_implemetation"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/image_rect_seg", 1, &imageCallback);
    // ros::Subscriber img_sub = nh.subscribe("/image_rect_seg", 1, imageCallback);

	TransformationGenerator beadsGenerator; // instaniate an object of TransformationGenerator

    ros::Duration sleep(0.5);
    // instaniate a camera projection matrix object
    cv_projective::cameraProjectionMatrices cam_proj_mat(nh, std::string("/camera/camera_info"), std::string("/camera/camera_info"));
    cv::Mat P_mat; // define a cv::Mat variable to store projection matrix

    // instaniate an variable of message of transformation_generator::ListOfPoints
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
        
        P_mat = cam_proj_mat.getLeftProjectionMatrix();

        particles_set_trans_mat_update.resize(particles_set_trans_mat.size());
        beads_pos_update.resize(particles_set_trans_mat.size());

        if (g_new_image) // if a new image is available, process it.
        {
            g_new_image = false;
            

            
            // for each particle
            for (int i = 0; i < N; ++i)
            {  
                // get one new particle from last step
                Eigen::Affine3d particle_trans_mat_update = beadsGenerator.getNewTransformationMatrix(particles_set_trans_mat[i], delta_time);
                ROS_INFO_STREAM("updated particle trans mat \n" << particle_trans_mat_update.matrix());
                // generate 9 beads position in camera frame
                beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points, particle_trans_mat_update);
                ROS_INFO_STREAM("beads position generated");
                // beads_pos_update[i] = list_of_points;
                int npts = list_of_points.points.size();

                // define the expected beads image mat for each particle
                cv::Mat expected_bead_pos_image(g_frame_in.size(), CV_8UC1);
                // the weight for each particle
                cv::Mat result_mat;
                    
                // for 9 beads;
                for (int k = 0; k < npts; ++k)
                {   
                    ROS_INFO("start to draw each bead at one time");
                    // convert ros geomertry/points to cv::Point3d
                    cv::Point3d bead_i_pos(list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);
                    ROS_INFO("bead center <%f, %f, %f>", list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);

                    ROS_INFO("start to render sphere");

                    cv::Rect bead_i_rendered = cv_3d::renderSphere(expected_bead_pos_image, cv_3d::sphere(bead_i_pos, RADIUS), P_mat);

                    ROS_INFO("one bead drawn");
                    // imshow("Black Beads", bead_i_rendered);
                }
                ROS_INFO_STREAM("finish beads drawing for one particle");
                imshow("beads drawing", expected_bead_pos_image);
                cv::matchTemplate(g_frame_in, expected_bead_pos_image, result_mat, CV_TM_CCOEFF_NORMED);
                ROS_INFO_STREAM("weight" << result_mat);
                // particles_set_trans_mat_update.push_back(Eigen::Affine3d particle_trans_mat_update);
                // beads_pos_update.push_back();
            }
        }

        sleep.sleep();
        ros::spinOnce();
    }
	return 0;
}