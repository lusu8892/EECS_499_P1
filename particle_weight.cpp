#include <particle_weight/particle_weight.h>

// Constructor
ParticleWeight::ParticleWeight(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
	ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
}

void ParticleWeight::initializePublishers()
{
	particle_weight_pub_ = nh_.advertise<std_msgs::Float64>("particle_weight", 1, true;
}

void ParticleWeight::initializeSubscribers()
{
	img_sub_ = nh_.subscribe("/image_rect_seg", 1, &ParticleWeight::imageCallback, this);
}

void ParticleWeight::imageCallback(const sensor_msgs::ImageConstPtr& segmented_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        
        cv_ptr = cv_bridge::toCvShare(segmented_image, CV_16UC1);
        // frameIn = cv_ptr->image.clone();
        // newImage = true;
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
}