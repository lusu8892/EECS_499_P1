#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_opencv_common/projective_geometry.h>

using namespace cv;
using namespace std;
using namespace cv_projective;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

Mat frameIn;
bool newImage;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvShare(original_image, enc::BGR8);
        frameIn = cv_ptr->image.clone();
        newImage = true;
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv )
{

    ros::init(argc, argv, "find_black_beads");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/image_rect", 1, &imageCallback);
      
    image_transport::Publisher img_pub = it.advertise("/image_rect_seg",1);

    newImage =false;

    cv::Point3d deprojection_output;

    Mat segmentedImage(100,100,CV_8UC1);

    while(ros::ok())
    {
        ros::spinOnce();
        
        if (newImage) // if a new image is available, process it.
        {
            newImage = false;

            // segment the image
            Mat frame_gray;


            cvtColor(frameIn, frame_gray, CV_BGR2GRAY);  // convert color image to gray image

            Mat darkHighlights = frame_gray * -1 + 255;

            morphologyEx(darkHighlights, segmentedImage, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", segmentedImage).toImageMsg();
            img_pub.publish(msg);
        }
        imshow("Black Beads", segmentedImage);
        char k(waitKey(5));
        if (k == 27 ) break;
    }
    return 0;
}