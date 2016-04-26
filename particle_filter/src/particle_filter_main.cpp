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
const int N = 2000; // The number of particles the system generates
const Eigen::Matrix3d EYE_3 = Eigen::MatrixXd::Identity(3,3);
const double RADIUS = 0.0082/2;


using namespace std;
using namespace cv_3d;

cv::Mat g_frame_in;
cv::Mat g_frame_normalized;
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
    // ROS_INFO_STREAM("Oe" << Oe);
    if (flag == "quaternion")
    {
        Eigen::Quaterniond q;
        Eigen::Quaterniond q_perturbation;
        q_perturbation.x() = (*func_ptr_1)(g,h);
        q_perturbation.y() = (*func_ptr_1)(g,h);
        q_perturbation.z() = (*func_ptr_1)(g,h);
        q_perturbation.w() = (*func_ptr_0)(i,j);
        q = q_perturbation.normalized() * input_q.normalized();
        Re = q.normalized().toRotationMatrix();
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
        // cv::normalize(g_frame_in, g_frame_normalized, 0, 255, NORM_L1);
        g_new_image = true;
    }
    catch (cv_bridge::Exception& e)
    {   
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
}

void particlesGeneration(vector<Eigen::Affine3d>& particles_set)
{   
    // particles_set.clear();
    Eigen::Affine3d initial_state = randomTransformationMatrixGenerator(getGaussianRandomNum, 
                                    getGaussianRandomNum, "no", 0, 0, 0, 0, 0.4, 0.01, 0, 0, 0, 0);
    
    initial_state.linear()(0,0) = -1; initial_state.linear()(2,2) = -1;

    // ROS_INFO_STREAM("initial state rotation part \n" << initial_state.linear());

    Eigen::Affine3d rot_mat_z = randomTransformationMatrixGenerator(getGaussianRandomNum, 
                                getGaussianRandomNum, "arbitrary", 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, Eigen::Vector3d(0,0,1));

    double theta = getGaussianRandomNum(0, 0.1*PI);
    Eigen::Vector3d rot_axis(cos(theta), sin(theta), 0);
    
    Eigen::Affine3d rot_mat_a = randomTransformationMatrixGenerator(getGaussianRandomNum, 
                            getGaussianRandomNum, "arbitrary", 0, 0, 0, 0, 0, 0, 0, 10/180, 0, 0, rot_axis);

    // paticle filter starts from there
    initial_state.linear() = rot_mat_a.linear() * rot_mat_z.linear() * initial_state.linear();

    // // convert Eigen::Affine3d transformation matrix to flat 4-by-4 Eigen::Matrix4d type
    // Eigen::Matrix4d initial_state_mat = initial_state.matrix();
    // ROS_INFO_STREAM("initial state mat \n" << initial_state.matrix());
    particles_set.push_back(initial_state);
}

void addNoiseToImage(cv::Mat& input_image)
{
    
    cv::Mat noise(input_image.size(),CV_8UC1);
    // normalize(imGray, result, 0.0, 1.0, CV_MINMAX, CV_64F);
    cv::randn(noise, 0, 0.05);
    input_image = cv::abs(input_image + noise);
    // cv::normalize(input_image, input_image, 1.0, 0.0, CV_MINMAX, CV_8UC1);
    // cv::imshow("OUTPUT",input_image);
    
    // cv::waitKey(10);
}

void lowVarianceSampler(const vector<Eigen::Affine3d>& particles_set_update, const vector<float>&  weight_vec,
                        vector<Eigen::Affine3d>& particles_set)
{   
    float weight_sum(0.0);
    vector<float> normd_weight_vec(weight_vec.size());
    // Normalize weight vector to form a probability distribution (i.e. sum to 1).
    for (int i = 0; i < weight_vec.size(); ++i)
    {
        
        weight_sum += weight_vec[i];
    }
    // ROS_INFO_STREAM("weight_vec size = " << weight_vec.size());
    for (int i = 0; i < normd_weight_vec.size(); ++i)
    {
        normd_weight_vec[i] = weight_vec[i] / weight_sum;
        ROS_INFO("Paritcle No. %d, normalized weight %f", i, normd_weight_vec[i]);

    }

    // implement low_variance_sampler
    float N_inv = (float)1/ float(N);
    float r = getUniformRandomNum(0, N_inv);
    // ROS_INFO("r = %f", r);
    float c = normd_weight_vec[0]; // get the first particle's weight
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
            c = c + normd_weight_vec[indx];
            // ROS_INFO("indx %d", indx);
        }
        // ROS_INFO("No. particle to be chosen %d", indx);
        particles_set.push_back(particles_set_update[indx]);
        // ROS_INFO_STREAM("mat \n" << particles_set_update[indx].matrix());
    }
}

void drawBeads(const transformation_generator::ListOfPoints& list_of_points, const cv::Mat& projection_matrix,
                cv::Mat& image_beads)
{
    int npts = list_of_points.points.size();

    // define the expected beads image mat for each particle
    // cv::Mat image_beads(100, 100, CV_8UC1);
    image_beads *= 0;
    int count(0); // the counter counting number of times failing to draw beads
    // for 9 beads;
    for (int k = 0; k < npts; ++k)
    {   
        // convert ros geomertry/points to cv::Point3d
        cv::Point3d bead_i_pos(list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);
        // ROS_INFO("bead center <%f, %f, %f>", list_of_points.points[k].x, list_of_points.points[k].y, list_of_points.points[k].z);

        // ROS_INFO("start to render sphere");
        if (bead_i_pos.z < 0)
        {
            count += 1;
            image_beads *= 1;
        }
        else
        {
            cv::Rect bead_i_rendered = cv_3d::renderSphere(image_beads, cv_3d::sphere(bead_i_pos, RADIUS), projection_matrix);    
        }
    }
    // if count equals to number of beads so the expected image will be left just as blank with white noise.
    if (count == npts)
    {
        image_beads *= 0;
        // addNoiseToImage(image_beads);
        // ROS_INFO("adding noise to blank image");
    }
    else
    {
        // addNoiseToImage(image_beads);
        // ROS_INFO("adding noise to good image");
    }
}

double imageCompareHist(const cv::Mat& src_base, const cv::Mat& src_test1)
{
    cv::Mat hsv_base;
    cv::Mat hsv_test1;


    /// Convert to HSV
    cv::cvtColor( src_base, hsv_base, CV_GRAY2RGB );
    cv::cvtColor( src_test1, hsv_test1, CV_GRAY2RGB );

    cv::cvtColor( hsv_base, hsv_base, CV_RGB2HSV );
    cv::cvtColor( hsv_test1, hsv_test1, CV_RGB2HSV );


    // hsv_half_down = hsv_base( Range( hsv_base.rows/2, hsv_base.rows - 1 ), Range( 0, hsv_base.cols - 1 ) );

    /// Using 50 bins for hue and 60 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };


    /// Histograms
    cv::MatND hist_base;
    cv::MatND hist_test1;

    /// Calculate the histograms for the HSV images
    cv::calcHist( &hsv_base, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
    cv::normalize( hist_base, hist_base, 0, 1, 32, -1, cv::Mat() );

    cv::calcHist( &hsv_test1, 1, channels, cv::Mat(), hist_test1, 2, histSize, ranges, true, false );
    cv::normalize( hist_test1, hist_test1, 0, 1, 32, -1, cv::Mat() );

    /// Apply the histogram comparison methods
    double weight = cv::compareHist( hist_base, hist_test1, CV_COMP_INTERSECT);

    // printf( "Done \n" );
    return weight;
}

void checkImageElement(const cv::Mat& input_image)
{
    for (int ij = 0; ij < 100; ++ij)
    {
        for (int ik= 0; ik < 100; ++ik)
        {
            if ((int) input_image.at<uchar>(ij, ik) > 0)
            {
                cout << "pixel value is = " << (int) input_image.at<uchar>(ij, ik) << endl;    
            }
                
        }
        
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

    ros::Duration sleep(5.0);    
    // instaniate a camera projection matrix object
    cv_projective::cameraProjectionMatrices cam_proj_mat(nh, 
            std::string("/camera/camera_info"), std::string("/camera/camera_info"));
    cv::Mat P_mat; // define a cv::Mat variable to store projection matrix
    P_mat = cam_proj_mat.getLeftProjectionMatrix();

    
    // instaniate an variable of message of transformation_generator::ListOfPoints
    transformation_generator::ListOfPoints list_of_points;
    transformation_generator::ListOfPoints ob_list_of_points;

    srand(time(NULL)); // random number seed;

    // make the randomly generated particles from the initial prior gaussian distribution
    // each partical is a transformation matrix from body frame to camera frame
    vector<Eigen::Affine3d> particles_set;

    vector<double> time_vec;
    for (int i = 0; i < 10000; ++i)
    {
        double time_increment = i * 1; // time increment is 1 second
        time_vec.push_back(time_increment);
    }

    for (int i = 0; i < N; ++i)
    {
        particlesGeneration(particles_set);
        // Eigen::Affine3d partical_trans_mat;
        // partical_trans_mat = randomTransformationMatrixGenerator(getUniformRandomNum, 
        //         getGaussianRandomNum, "quaternion", -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.1, 0.1, 0.9, 1, Eigen::Vector3d(0,0,0), initial_state_mat);
        // particles_set.push_back(partical_trans_mat);
        // // ROS_INFO_STREAM("partical initial trans mat \n" << partical_trans_mat.matrix());
    }

    vector<Eigen::Affine3d> particles_set_update;
    vector<transformation_generator::ListOfPoints> beads_pos_update;
    vector<float>  weight_vec;

    double delta_time(0.0);
    Eigen::Affine3d observed_trans;
    observed_trans.matrix() = Eigen::MatrixXd::Identity(4,4);
    observed_trans.linear()(0,0) = -1; observed_trans.linear()(2,2) = -1;
    observed_trans.translation() << 0,0,0.4;
    while(ros::ok())
    {   
        for(int j = 0; j < time_vec.size() - 1; ++j)
        {  
            // ros::spinOnce();
            ROS_INFO("iteration = %d", j);
            // if (g_new_image)
            // {
            //     g_new_image = false;
                if (j == 0)
                {
                    delta_time = 0;
                }
                else
                {
                    delta_time = time_vec[j+5] - time_vec[j];
                }
                
                P_mat = cam_proj_mat.getLeftProjectionMatrix();
                ROS_INFO_STREAM(P_mat.at<double>(0,0));
                while (P_mat.at<double>(0,0) <= 0)
                {
                    ros::spinOnce();
                    P_mat = cam_proj_mat.getLeftProjectionMatrix();
                    // ROS_INFO("to get current projection matrix");
                }
                // ROS_INFO_STREAM("projection matrix" << P_mat[1].size());

                particles_set_update.clear();
                // beads_pos_update.clear();
                weight_vec.clear();
                ROS_INFO("delta_time = %f", delta_time);
                // cv::Mat oberved_beads_image(100, 100, CV_8UC1);
                Eigen::Affine3d observed_trans_updated = beadsGenerator.getNewTransformationMatrix(observed_trans, "hybrid", delta_time);
                // ROS_INFO_STREAM("updated observed transformation matrix \n" << observed_trans_updated.matrix() << "\n");

                beadsGenerator.getBeadsPosition(9, 3, 3, ob_list_of_points, observed_trans_updated);
                cv::Mat oberved_beads_image(480,1000, CV_8UC1);
                drawBeads(ob_list_of_points, P_mat, oberved_beads_image);
                // cv::imshow("particle filter", oberved_beads_image);
                // cv::waitKey(10);

                // for each particle
                for (int i = 0; i < N; ++i)
                {
                    // get one new particle from last step
                    Eigen::Affine3d particle_trans_mat_update = beadsGenerator.getNewTransformationMatrix(particles_set[i], "body", delta_time);
                    // ROS_INFO_STREAM("updated particle trans mat \n" << particle_trans_mat_update.matrix());
                    // generate 9 beads position in camera frame
                    beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points, particle_trans_mat_update);
                    // ROS_INFO_STREAM("beads position generated");
                    // beads_pos_update[i] = list_of_points
                    cv::Mat expected_beads_image(oberved_beads_image.size(), CV_8UC1);
                    drawBeads(list_of_points, P_mat, expected_beads_image);
                    // define a mat to store the two expected and observed image
                    cv::Mat blended_image(expected_beads_image.size(), CV_8UC1);
                    
                    // define the expected beads image mat for each particle
                    cv::Mat weight; weight *= 0;

                    cv::addWeighted(expected_beads_image, 0.7, oberved_beads_image, 0.3, 0.0, blended_image);
                    cv::imshow("particle filter", blended_image);
                    cv::waitKey(10);
                    cv::matchTemplate(oberved_beads_image, expected_beads_image, weight, CV_TM_CCOEFF_NORMED);

                    // weight_vec.push_back(imageCompareHist(expected_beads_image, oberved_beads_image));
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
                observed_trans = observed_trans_updated;
            // }
            ros::spinOnce();
        }
        
    }
    return 0;
}
    



	// while(ros::ok())
	// {   
 //        // ros::console::shutdown();
 //        // loop_begin = ros::Time::now().toSec();
 //        // // delta_time = loop_end - loop_begin;
 //        // ROS_INFO("delta_time = %f", delta_time);

 //        if (g_new_image) // if a new image is available, process it.
 //        {
        
 //            P_mat = cam_proj_mat.getLeftProjectionMatrix();

 //            particles_set_update.clear();
 //            beads_pos_update.clear();
 //            weight_vec.clear();
 //            // weight_sum.release(); // clear weight_sum;

 //            g_new_image = false;

 //            // int count = 0;
 //            loop_begin = ros::Time::now().toSec();
 //            ROS_INFO("delta_time = %f", delta_time);
 //            // begin = ros::Time::now().toSec();
 //            // for each particle
 //            for (int i = 0; i < N; ++i)
 //            {
 //                // get one new particle from last step
 //                Eigen::Affine3d particle_trans_mat_update = beadsGenerator.getNewTransformationMatrix(particles_set[i], "body", delta_time);
 //                // ROS_INFO_STREAM("updated particle trans mat \n" << particle_trans_mat_update.matrix());
 //                // generate 9 beads position in camera frame
 //                beadsGenerator.getBeadsPosition(9, 3, 3, list_of_points, particle_trans_mat_update);
 //                // ROS_INFO_STREAM("beads position generated");
 //                // beads_pos_update[i] = list_of_points;
 //                int npts = list_of_points.points.size();

 //                // define the expected beads image mat for each particle
 //                cv::Mat expected_beads_image(g_frame_in.size(), CV_8UC1);
 //                drawBeads(list_of_points, P_mat, expected_beads_image);
            
 //                // define a mat to store the two expected and observed image
 //                cv::Mat blended_image(g_frame_in.size(), CV_8UC1);
            
 //                // define the expected beads image mat for each particle
 //                cv::Mat weight;
 //                weight *= 0;
                
 //                checkImageElement(g_frame_in);

 //                cv::addWeighted(expected_beads_image, 0.7, g_frame_in, 0.3, 0.0, blended_image);
 //                cv::imshow("particle filter", blended_image);
 //                cv::waitKey(10);
 //                cv::matchTemplate(g_frame_in, expected_beads_image, weight, CV_TM_CCOEFF_NORMED);
 //                weight_vec.push_back(weight.at<float>(0,0)); // convert cv::Mat to float number and push back 
 //                // ROS_INFO_STREAM("weight =" << weight.at<float>(0,0));
 //                particles_set_update.push_back(particle_trans_mat_update);
 //                // beads_pos_update.push_back();
 //                // count += 1;
 //                // ROS_INFO("numbers of particle generated = %d", count);
 //            }
 //            lowVarianceSampler(particles_set_update, weight_vec, particles_set);
 //            // cout << "indx = " << indx << endl;
 //            ROS_INFO("particle set size = %d", (int)particles_set.size());
 //            // loop_end = ros::Time::now().toSec();
 //            loop_end = ros::Time::now().toSec();
 //            delta_time = loop_end - loop_begin;
 //        }
 //        // loop_end = ros::Time::now().toSec();
 //        // delta_time = loop_end - loop_begin;

 //        // sleep.sleep();
 //        // loop_end = ros::Time::now().toSec();
 //        // delta_time = loop_end - loop_begin;
 //        ros::spinOnce();
 //        // ROS_INFO("delta_time = %f", delta_time);
 //    }
// 	return 0;
// }