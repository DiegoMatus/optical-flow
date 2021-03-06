/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src2, imgA, imgB;
vector<Point2f> corners;
int win_size = 10;
int maxCorners = 23;
int maxTrackbar = 500;

IplImage* img1 = cvLoadImage( "frame1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
IplImage* img2 = cvLoadImage( "frame2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
CvPoint2D32f* cornersA = new CvPoint2D32f[ maxCorners ];

RNG rng(12345);
const char* source_window = "Image";

/// Function header
void goodFeaturesToTrack_Demo( int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{

  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  //namedWindow("edges",1);
  for(;;)
  {
    Mat frame;
    Mat frame2;
    cap >> frame; // get a new frame from camera
    cap >> frame2; // get a new frame from camera
    cvtColor(frame, src, CV_BGR2GRAY);
    cvtColor(frame2, src2, CV_BGR2GRAY);
    /// Load source image and convert it to gray

    IplImage* imgC = cvLoadImage( "OpticalFlow1.jpg",CV_LOAD_IMAGE_UNCHANGED);
    /// Create Window
    namedWindow( source_window, WINDOW_AUTOSIZE );

    /// Create Trackbar to set the number of corners
    createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo );

    GaussianBlur(source_window, source_window, Size(7,7), 1.5, 1.5);
    Canny(source_window, source_window, 0, 30, 3);
    imshow( source_window, src );

    goodFeaturesToTrack_Demo( 0, 0 );

    cvFindCornerSubPix(
      img1,
      cornersA,
      maxCorners,
      cvSize(win_size,win_size),
      cvSize(-1, 1),
      cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03)
      if(waitKey(30) >= 0) break;
    );
    

  //waitKey(0);
  return(0);
}

/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void goodFeaturesToTrack_Demo( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src.clone();

  /// Apply corner detection
  goodFeaturesToTrack( imgA,
               cornersA,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );
}
