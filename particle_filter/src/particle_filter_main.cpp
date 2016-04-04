// paticle_filter_main.cpp //
/// This is the main function of the implementation of particle filter to track
/// the beads position based on the segmented image source
/// the particle are presented as transformation matrix from body to camera frame
/// based on each particle, 9 beads were created by opencv function

#include <ros/ros.h>
#include <ros/console.h>
#include <time.h>
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
const int N = 100; // The number of particles the system generates
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
Eigen::Affine3d randomTransformationMatrixGenerator(double (*func_ptr_0)(double, double),
                double (*func_ptr_1)(double, double),
                const std::string& flag, 
                double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, 
                double f = 0, double g = 0, double h = 0, double i = 0, double j = 0,
                const Eigen::Vector3d& rotate_axis = Eigen::Vector3d(0,0,0),
                const Eigen::Matrix4d& input_trans_mat = Eigen::MatrixXd::Identity(4,4))
{
    Eigen::Affine3d random_trans_mat;
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;

    Eigen::Affine3d temp_trans_mat;

    // ROS_INFO_STREAM("the input trans mat \n" << input_trans_mat.matrix());

    temp_trans_mat.matrix() = input_trans_mat;
    
    Eigen::Vector3d input_translation = temp_trans_mat.translation();
    // cout << "translation part " << temp_trans_mat.translation() << " " << endl;
    Eigen::Quaterniond input_q(temp_trans_mat.linear());

    Oe(0)= (*func_ptr_0)(a,b) + input_translation(0);
    Oe(1)= (*func_ptr_0)(c,d) + input_translation(1);
    Oe(2)= (*func_ptr_0)(e,f) + input_translation(2);
    random_trans_mat.translation() = Oe; // the "translation" part of affine is the vector between origins

    if (flag == "quaternion")
    {
        Eigen::Quaterniond q;
        Eigen::Quaterniond q_perturbation;
        q_perturbation.x() = (*func_ptr_1)(g,h);
        q_perturbation.y() = (*func_ptr_1)(g,h);
        q_perturbation.z() = (*func_ptr_1)(g,h);
        q_perturbation.w() = (*func_ptr_0)(i,j);
        // Eigen::Quaterniond<Scalar> 
        // double magnitude;
        // q.x() = (*func_ptr_1)(g,h) + input_q.x();
        // q.y() = (*func_ptr_1)(g,h) + input_q.y();
        // q.z() = (*func_ptr_1)(g,h) + input_q.z();
        // q.w() = (*func_ptr_1)(g,h) + input_q.w();
        q = q_perturbation.normalized() * input_q.normalized();
        Re = q.normalized().toRotationMatrix();
        // Eigen::Matrix3d Re(q.normalized()); //convenient conversion...initialize a 3x3 orientation matrix
        // using a quaternion, q
        random_trans_mat.linear() = Re; // the rotational part of affine is the "linear" part
    }
    else if (flag == "arbitrary")
    {
        Re = Eigen::AngleAxisd((*func_ptr_1)(g,h)*PI, rotate_axis.normalized());
        random_trans_mat.linear() = Re;
    }
    else
    {
        Re = Eigen::Matrix3d::Identity(3,3);
        random_trans_mat.linear() = Re;
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

void addNoiseToImage(cv::Mat& input_image)
{
    
    cv::Mat noise(input_image.size(),CV_8UC1);
    // normalize(imGray, result, 0.0, 1.0, CV_MINMAX, CV_64F);
    cv::randn(noise, 0, 0.05);
    input_image = input_image + cv::abs(noise);
    // cv::normalize(input_image, input_image, 0.0, 1.0, CV_MINMAX, CV_8UC1);
    // cv::imshow("OUTPUT",input_image);
}

void lowVarianceSampler(const vector<Eigen::Affine3d>& particles_set_update, const vector<float>&  weight_vec,
                        vector<Eigen::Affine3d>& particles_set)
{   
    float weight_sum(0.0);
    vector<float> normd_weight_vec(weight_vec.size());
    // ROS_INFO("numbers of particle updated %d", (int)particles_set_update.size());
    // Normalize weight vector to form a probability distribution (i.e. sum to 1).
    for (int i = 0; i < weight_vec.size(); ++i)
    {
        weight_sum += weight_vec[i];
    }
    // ROS_INFO_STREAM("weight_vec size = " << weight_vec.size());
    for (int i = 0; i < normd_weight_vec.size(); ++i)
    {
        normd_weight_vec[i] = weight_vec[i] / weight_sum;     
    }

    // weight_sum = 0; // set wet sum to zero
    // for (vector<float>::iterator it = normd_weight_vec.begin(); it < normd_weight_vec.end(); ++it)
    // {
    //     weight_sum += *it;
    // }
    // ROS_INFO_STREAM("normalized weight sum = " << weight_sum);

    // implement low_variance_sampler
    float N_inv = (float)1/ float(N);
    float r = getUniformRandomNum(0, N_inv);
    ROS_INFO("r = %f", r);
    float c = weight_vec[0]; // get the first particle's weight
    // ROS_INFO("fisrt weight %f", c);
    int indx(0);
    float U(0.0);
    particles_set.clear();
    for (int m = 0; m < N; ++m)
    {
        U = r + (float)m * N_inv;
        // ROS_INFO("U = %f", U);
        while (U > c)
        {
            indx = indx + 1;
            c = c + weight_vec[indx];
            ROS_INFO("indx %d", indx);
        }
        
        particles_set.push_back(particles_set_update[indx]);
        // ROS_INFO_STREAM("mat \n" << particles_set_update[indx].matrix());
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter_implemetation"); //node name
	ros::NodeHandle nh; // don't really need this in this example
	ros::Publisher beads_pos_pub = nh.advertise<transformation_generator::ListOfPoints>("beads_random_position", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/image_rect_seg", 1, &imageCallback);
    // ros::Subscriber img_sub = nh.subscribe("/image_rect_seg", 1, imageCallback);

    cv::namedWindow( "particle filter", 1 );
	TransformationGenerator beadsGenerator; // instaniate an object of TransformationGenerator

    ros::Duration sleep(0.1);
    // instaniate a camera projection matrix object
    cv_projective::cameraProjectionMatrices cam_proj_mat(nh, 
            std::string("/camera/camera_info"), std::string("/camera/camera_info"));
    cv::Mat P_mat; // define a cv::Mat variable to store projection matrix

    // instaniate an variable of message of transformation_generator::ListOfPoints
	transformation_generator::ListOfPoints list_of_points;

	srand(time(NULL)); // random number seed;

    Eigen::Affine3d initial_state = randomTransformationMatrixGenerator(getUniformRandomNum, 
                                    getUniformRandomNum, "no", -0.3, 0.3, -0.2, 0.2, 0.3, 0.8, 0, 0,0,0);
    
    initial_state.linear()(0,0) = -1; initial_state.linear()(2,2) = -1;

    ROS_INFO_STREAM("initial state rotation part \n" << initial_state.linear());

    Eigen::Affine3d rot_mat_z = randomTransformationMatrixGenerator(getUniformRandomNum, 
                                getUniformRandomNum, "arbitrary", 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, Eigen::Vector3d(0,0,1));

    double theta = getUniformRandomNum(0, 2*PI);
    Eigen::Vector3d rot_axis(cos(theta), sin(theta), 0);
    
    Eigen::Affine3d rot_mat_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 
                            getGaussianRandomNum, "arbitrary", 0, 0, 0, 0, 0, 0, 0, 10/180, 0, 0, rot_axis);

    // paticle filter starts from there
    initial_state.linear() = rot_mat_a.linear() * rot_mat_z.linear() * initial_state.linear();

    // convert Eigen::Affine3d transformation matrix to flat 4-by-4 Eigen::Matrix4d type
    Eigen::Matrix4d initial_state_mat = initial_state.matrix();
    ROS_INFO_STREAM("initial state mat \n" << initial_state.matrix());

    // make the randomly generated particles from the initial prior gaussian distribution
    // each partical is a transformation matrix from body frame to camera frame
    vector<Eigen::Affine3d> particles_set;
    for (int i = 0; i < N; ++i)
    {
        Eigen::Affine3d partical_trans_mat;
        partical_trans_mat = randomTransformationMatrixGenerator(getUniformRandomNum, 
                getGaussianRandomNum, "quaternion", -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.1, 0.1, 0.9, 1, Eigen::Vector3d(0,0,0), initial_state_mat);
        particles_set.push_back(partical_trans_mat);
        // ROS_INFO_STREAM("partical initial trans mat \n" << partical_trans_mat.matrix());

    }

    vector<Eigen::Affine3d> particles_set_update;
    vector<transformation_generator::ListOfPoints> beads_pos_update;
    vector<float>  weight_vec;
    // float weight_sum;
    
    // double delta_time = 0.02;
    double loop_begin(ros::Time::now().toSec()); // set a time for recording the start time of each iteration
    double loop_end(ros::Time::now().toSec()); // set a time for recording end time of each iteration

    double delta_time(0.0);

    // ROS_INFO("delta_time = %f", delta_time);

	while(ros::ok())
	{   
        // ros::console::shutdown();
        // loop_begin = ros::Time::now().toSec();
        // // delta_time = loop_end - loop_begin;
        // ROS_INFO("delta_time = %f", delta_time);

        if (g_new_image) // if a new image is available, process it.
        {
            
            P_mat = cam_proj_mat.getLeftProjectionMatrix();

            particles_set_update.clear();
            beads_pos_update.clear();
            weight_vec.clear();
            // weight_sum.release(); // clear weight_sum;

            g_new_image = false;

            // int count = 0;
            loop_begin = ros::Time::now().toSec();
            ROS_INFO("delta_time = %f", delta_time);
            // begin = ros::Time::now().toSec();
            // for each particle
            for (int i = 0; i < N; ++i)
            {
                // get one new particle from last step
                Eigen::Affine3d particle_trans_mat_update = beadsGenerator.getNewTransformationMatrix(particles_set[i], delta_time);
                // ROS_INFO_STREAM("updated particle trans mat \n" << particle_trans_mat_update.matrix());
                // generate 9 beads position in camera frame
                beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points, particle_trans_mat_update);
                // ROS_INFO_STREAM("beads position generated");
                // beads_pos_update[i] = list_of_points;
                int npts = list_of_points.points.size();

                // define the expected beads image mat for each particle
                cv::Mat expected_bead_pos_image(g_frame_in.size(), CV_8UC1);
                expected_bead_pos_image *= 0;
                
                // define a mat to store the two expected and observed image
                cv::Mat blended_image(g_frame_in.size(), CV_8UC1);
                
                // define the expected beads image mat for each particle
                cv::Mat weight;
                weight *= 0;

                int count(0); // the counter counting number of times failing to draw beads
                // for 9 beads;
                for (int k = 0; k < npts; ++k)
                {   
                    // int count(0); // the counter counting number of times failing to draw beads
                    // ROS_INFO("start to draw each bead at one time");
                    // convert ros geomertry/points to cv::Point3d
                    cv::Point3d bead_i_pos(list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);
                    // ROS_INFO("bead center <%f, %f, %f>", list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);

                    // ROS_INFO("start to render sphere");
                    if (bead_i_pos.z < 0)
                    {
                        count += 1;
                        expected_bead_pos_image *= 1;
                    }
                    else
                    {
                        cv::Rect bead_i_rendered = cv_3d::renderSphere(expected_bead_pos_image, cv_3d::sphere(bead_i_pos, RADIUS), P_mat);    
                    }

                    // ROS_INFO("one bead drawn");
                    // imshow("Black Beads", bead_i_rendered);
                }
                // if count equals to number of beads so the expected image will be left just as blank with white noise.
                if (count == npts)
                {
                    expected_bead_pos_image *= 0;
                    addNoiseToImage(expected_bead_pos_image);
                    // ROS_INFO("adding noise to blank image");
                }
                else
                {
                    addNoiseToImage(expected_bead_pos_image);
                    // ROS_INFO("adding noise to good image");
                }

                cv::addWeighted(expected_bead_pos_image, 0.7, g_frame_in, 0.3, 0.0, blended_image);
                cv::imshow("particle filter", blended_image);
                cv::waitKey(10);
                cv::matchTemplate(g_frame_in, expected_bead_pos_image, weight, CV_TM_CCOEFF_NORMED);
                weight_vec.push_back(weight.at<float>(0,0)); // convert cv::Mat to float number and push back 
                // ROS_INFO_STREAM("weight =" << weight.at<float>(0,0));
                particles_set_update.push_back(particle_trans_mat_update);
                // beads_pos_update.push_back();
                // count += 1;
                // ROS_INFO("numbers of particle generated = %d", count);
            }
            lowVarianceSampler(particles_set_update, weight_vec, particles_set);
            // cout << "indx = " << indx << endl;
            ROS_INFO("particle set size = %d", (int)particles_set.size());
            // loop_end = ros::Time::now().toSec();
            loop_end = ros::Time::now().toSec();
            delta_time = loop_end - loop_begin;
        }
        // loop_end = ros::Time::now().toSec();
        // delta_time = loop_end - loop_begin;

        // sleep.sleep();
        // loop_end = ros::Time::now().toSec();
        // delta_time = loop_end - loop_begin;
        ros::spinOnce();
        // ROS_INFO("delta_time = %f", delta_time);
    }
	return 0;
}