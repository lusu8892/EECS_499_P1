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

    Mat frame;
    Mat cvt_frame;
    Mat threshold_frame;
    Mat morph_frame;

	vector<vector<Point> > contours;
    vector<vector<Point> > modifed_contours;

    vector<Vec4i> hierarchy;

    for(int i=0 ;; i++)
    {
        frame.release();
        cvt_frame.release();
        threshold_frame.release();
        morph_frame.release();

        contours.clear();
        modifed_contours.clear();

        hierarchy.clear();

        cap >> frame; // get a new frame from camera
        cvtColor(frame, cvt_frame, CV_BGR2GRAY); // convert color image to gray image
        // imshow("Black Beads", frame);
        //if(waitKey(30) >= 0) break;
        threshold(cvt_frame, threshold_frame, 150, 255, THRESH_BINARY_INV+THRESH_OTSU); // segmentation
        
        morphologyEx(threshold_frame, morph_frame, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE,Size(3,3)));
        // imshow("Black Beads", morph_frame);
        // if(waitKey(30) >= 0) break;
        
        findContours(morph_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        // search and store contours which its area is btw certain value
        for (int j=0; j<contours.size();++j)
        { 
            double area;
            area = contourArea(contours[j]);
            if (area > 450 && area < 800)
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