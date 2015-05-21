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
Mat src, src2, src_gray1, src_gray2;

int maxCorners = 23;
int maxTrackbar = 100;

RNG rng(12345);
const char* source_window = "Image";

/// Function header
void opticalFlow( int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );

  cvtColor( src, src_gray1, COLOR_BGR2GRAY );
  cvtColor( src, src_gray2, COLOR_BGR2GRAY );

  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  imshow( source_window, src );

  opticalFlow( 0, 0 );

  waitKey(0);
  return(0);
}

/**
 * @function opticalFlow.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void opticalFlow( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray1,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


  /// Found the corners
  int win_size = 10;

  cornerSubPix(
    src_gray1,
    corners,
    cvSize(win_size,win_size),
    cvSize(-1, 1),
    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03)
  );

  ///Call the Lucas Kanade algorith
  /*calcOpticalFlowPyrLK(
      src_gray1,
      src_gray2,
      pyrA,
      pyrB,
      cornersA,
      cornersB,
      corner_count,
      cvSize( win_size, win_size),
      5,
      features_found,
      feature_errors,
      cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3),
      0
  );*/

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );
}
