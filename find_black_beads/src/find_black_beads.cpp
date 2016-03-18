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

using namespace cv;
using namespace std;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
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

    // Get the moments
    vector<Moments> mu(modifed_contours.size() );
    for ( int k = 0; k < modifed_contours.size(); k++ )
    {
        mu[k] = moments( modifed_contours[k], false );
    }

    // Get the mass centers:
    vector<Point2f> mc( modifed_contours.size() );
    for ( int k = 0; k < modifed_contours.size(); k++ )
    {
        mc[k] = Point2f( mu[k].m10/mu[k].m00 , mu[k].m01/mu[k].m00 );
        printf("centroid position: x = %f, y = %f \n", mc[k].x, mc[k].y);
    } 

    Mat drawing = Mat::zeros( morph_frame.size(), CV_8UC3 );

    for( int k = 0; k< modifed_contours.size(); k++ )
    {
        Scalar color = Scalar( 255, 255, 255 );
        
        drawContours( drawing, modifed_contours, k, color, 2, 8, hierarchy, 0, Point() );
    }
    
            
    imshow("Black Beads Motion", drawing);
    if(waitKey(30) >= 0) break;
    writer.write(drawing);
     
 
    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv )
{

    ros::init(argc, argv, "find_black_beads");
    ros::NodeHandle n; //

    //Use method of ImageTransport to create image publisher
    image_transport::Publisher pub;
 

 //    VideoCapture cap(0);
 //    if(cap.isOpened()){
 //      printf("Camera Open \n");
 //    } 

 //    VideoWriter writer; // instantiate a video recorder object

 //    // filename string
 //    string video_filename = "BlackBeadsRandoMotion.avi";
 //    // fourcc integer
 //    int fcc = CV_FOURCC('D','I','V','X');
 //    // frames per sec integer
 //    int fps = 10;
 //    // frame size 
 //    cv::Size frame_size(cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT));

 //    writer = VideoWriter(video_filename, fcc, fps, frame_size); // initializing an object of ViderWriter

 //    if(!writer.isOpened())
 //    {
 //        cout << "ERROR OPENING FILE FOR WRITE" << endl;
 //        getchar();

 //        return -1;
 //    }



 //    namedWindow("Black Beads Random Motion", WINDOW_AUTOSIZE );
 //    string file_name;
 //    char buffer [50];

 //    Mat frame;
 //    Mat cvt_frame;
 //    Mat threshold_frame;
 //    Mat morph_frame;

	// vector<vector<Point> > contours;
 //    vector<vector<Point> > modifed_contours;

 //    vector<Vec4i> hierarchy;

 //    for(int i=0 ;; i++)
 //    {
 //        frame.release();
 //        cvt_frame.release();
 //        threshold_frame.release();
 //        morph_frame.release();

 //        contours.clear();
 //        modifed_contours.clear();

 //        hierarchy.clear();

 //        cap >> frame; // get a new frame from camera
 //        cvtColor(frame, cvt_frame, CV_BGR2GRAY); // convert color image to gray image
 //        // imshow("Black Beads", frame);
 //        //if(waitKey(30) >= 0) break;
 //        threshold(cvt_frame, threshold_frame, 50, 255, THRESH_BINARY_INV); // segmentation
 //        // imshow("Black Beads", threshold_frame);
 //        // if(waitKey(30) >= 0) break;

 //        morphologyEx(threshold_frame, morph_frame, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE,Size(3,3)));
 //        // imshow("Black Beads", morph_frame);
 //        // if(waitKey(30) >= 0) break;
        
 //        findContours(morph_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

 //        // search and store contours which its area is btw certain value
 //        for (int j=0; j<contours.size();++j)
 //        { 
 //            double area;
 //            area = contourArea(contours[j]);
 //            if (area > 350 && area < 800)
 //            {
 //                modifed_contours.push_back(contours[j]);
 //                printf("Area: %f \n",area);

 //            }
 //        }

 //        cout << modifed_contours.size() << endl;

 //        // Get the moments
 //        vector<Moments> mu(modifed_contours.size() );
 //        for ( int k = 0; k < modifed_contours.size(); k++ )
 //        {
 //            mu[k] = moments( modifed_contours[k], false );
 //        }

 //        // Get the mass centers:
 //        vector<Point2f> mc( modifed_contours.size() );
 //        for ( int k = 0; k < modifed_contours.size(); k++ )
 //        {
 //            mc[k] = Point2f( mu[k].m10/mu[k].m00 , mu[k].m01/mu[k].m00 );
 //            printf("centroid position: x = %f, y = %f \n", mc[k].x, mc[k].y);
 //        } 

 //        Mat drawing = Mat::zeros( morph_frame.size(), CV_8UC3 );

 //       	for( int k = 0; k< modifed_contours.size(); k++ )
 //       	{
 //            Scalar color = Scalar( 255, 255, 255 );
            
 //           	drawContours( drawing, modifed_contours, k, color, 2, 8, hierarchy, 0, Point() );
 //       	}
        
                
 //        imshow("Black Beads Motion", drawing);
 //        if(waitKey(30) >= 0) break;
 //        writer.write(drawing);
 //       	// printf("here");
 //       	// fitEllipse(contours);
 //       	// printf("finish fitting");
        
 //        // file_name = sprintf(buffer, "%d.png",i);
 //        // imwrite(file_name, contours);
 //        // imshow("Black Beads", frame);
 //        // if(waitKey(30) >= 0) break;
 //    }
    
 //    waitKey(0);

    return 0;
}