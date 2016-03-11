#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>


using namespace cv;
using namespace std;
RNG rng(12345);


int main(int argc, char** argv )
{
    
    VideoCapture cap(0);
    if(cap.isOpened()){
      printf("Camera Open \n");
    } 

    
    namedWindow("Black Beads", WINDOW_AUTOSIZE );
    string file_name;
    char buffer [50];


    for(int i=0 ;; i++)
    {
        Mat frame;

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        cap >> frame; // get a new frame from camera
        cvtColor(frame, frame, CV_BGR2GRAY); // convert color image to gray image
        // imshow("Black Beads", frame);
        //if(waitKey(30) >= 0) break;
        threshold(frame, frame, 150, 255, THRESH_BINARY_INV+THRESH_OTSU); // segmentation
        
        morphologyEx(frame, frame, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE,Size(3,3)));
        imshow("Black Beads", frame);
        if(waitKey(30) >= 0) break;
        findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        cout<<contours.size();
        // Mat drawing = Mat::zeros( frame.size(), CV_8UC3 );
       	// Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       	// for( int k = 0; k< contours.size(); k++ )
       	// {
        //     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            
        //    	drawContours( drawing, contours, k, color, 2, 8, hierarchy, 0, Point() );
        //    	printf("dddd \n");
       	// }
        
       	// printf("here");
       	// fitEllipse(contours);
       	// printf("finish fitting");
        
        // file_name = sprintf(buffer, "%d.png",i);
        // imwrite(file_name, contours);
        // imshow("Black Beads", frame);
        // if(waitKey(30) >= 0) break;
    }
    
    waitKey(0);

    return 0;
}