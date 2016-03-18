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

Mat fameIn;
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
    
    Mat frame = cv_ptr->image.clone();
    Mat cvt_frame;
    Mat threshold_frame;
    Mat morph_frame;

    vector<vector<Point> > contours;
    vector<vector<Point> > modifed_contours;

    vector<Vec4i> hierarchy;

    cvtColor(frame, cvt_frame, CV_BGR2GRAY); // convert color image to gray image
    threshold(cvt_frame, threshold_frame, 50, 255, THRESH_BINARY_INV); // segmentation
    // imshow("Black Beads", threshold_frame);
    // if(waitKey(30) >= 0) break;

    morphologyEx(threshold_frame, morph_frame, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE,Size(3,3)));

    
    findContours(morph_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    // search and store contours which its area is btw certain value
    for (int j=0; j<contours.size();++j)
    { 
        double area;
        area = contourArea(contours[j]);
        if (area > 350 && area < 800)
        {
            modifed_contours.push_back(contours[j]);
            printf("Area: %f \n",area);

        }
    }

    cout << modifed_contours.size() << endl;

    Mat drawing = Mat::zeros( morph_frame.size(), CV_8UC3 );

    for( int k = 0; k< modifed_contours.size(); k++ )
    {
        Scalar color = Scalar( 255, 255, 255 );
        
        drawContours( drawing, modifed_contours, k, color, 2, 8, hierarchy, 0, Point() );
    }
    
            
    imshow("Black Beads Motion", drawing);
    // writer.write(drawing);
     
    //Display the image using OpenCV
    cv::imshow(WINDOW, drawing);
    waitKey(30);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    // cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    cv_ptr->image = drawing;
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv )
{

    ros::init(argc, argv, "find_black_beads");
    ros::NodeHandle nh;

    cameraProjectionMatrices cameraProjectionMatrices(nh, "", "");

    cv::Mat left_camera_image;
    cv::Mat right_camera_image;

    cv::Point3d deprojection_output;

    while(ros::ok())
    {
        ros::spinOnce();
        cameraProjectionMatrices.projectionSubscriptionCb(cv_ptr->toImageMsg(),0); // left camera
        cameraProjectionMatrices.projectionSubscriptionCb(cv_ptr->toImageMsg(),1); // right camera

        left_camera_image = cameraProjectionMatrices.getLeftProjectionMatrix(); // get left camera image
        right_camera_image = cameraProjectionMatrices.getRightProjectionMatrix(); // get right camera image

        deprojection_output = cameraProjectionMatrices.deprojectStereoPoint(left_camera_image,right_camera_image,P_l,P_r);


    }
    
    
 

 

    return 0;
}