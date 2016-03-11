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
        Mat cvt_frame;
        Mat threshold_frame;
        Mat morph_frame;

        vector<vector<Point> > contours;
        vector<vector<Point> > modifed_contours;

        vector<Vec4i> hierarchy;
        vector<double> area_vec;

        cap >> frame; // get a new frame from camera
        cvtColor(frame, cvt_frame, CV_BGR2GRAY); // convert color image to gray image
        // imshow("Black Beads", frame);
        //if(waitKey(30) >= 0) break;
        threshold(cvt_frame, threshold_frame, 150, 255, THRESH_BINARY_INV+THRESH_OTSU); // segmentation
        
        morphologyEx(threshold_frame, morph_frame, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE,Size(3,3)));
        imshow("Black Beads", morph_frame);
        if(waitKey(30) >= 0) break;
        
        findContours(morph_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        for (int j=0; j<contours.size();++j)
        { 
            double area;
            area = contourArea(contours[j]);
            if (area > 80 && area < 150)
            {
                modifed_contours.push_back(contours[j]);
            }
        }

        cout << modifed_contours.size() << endl;
        Mat drawing = Mat::zeros( morph_frame.size(), CV_8UC3 );

       	for( int k = 0; k< modifed_contours.size(); k++ )
       	{
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            
           	drawContours( drawing, modifed_contours, k, color, 2, 8, hierarchy, 0, Point() );
       	}
        
        imshow("Black Beads", drawing);
        if(waitKey(30) >= 0) break;
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