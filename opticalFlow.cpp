/**
 * @Optical flow
 * @brief algoritmh for detecting corners and optical flow using Shi-Tomasi and Lucas-Kanade methods
 * @author Matus Perdomo Diego
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "functions.h"
#include <iostream>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src2, src_gray1, src_gray2;


int maxCorners = 350;
int maxTrackbar = 500;

const char* source_window = "ImageA";
const char* source_windowB = "ImageB";
const char* source_windowR = "Results";

/// Function header
void opticalFlow( int, void* );
void canalX(Size, char *, Mat *, Mat *, Mat *, Mat *);

/**
 * @function main
 */
int main( int, char** argv ){
  int op = atoi(argv[3]);
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );
  //Size tamanio = src.size();

  //cvtColor( src, src_gray1, COLOR_BGR2GRAY );

  //canalX(tamanio, argv[3], &src, &src2, &src_gray1, &src_gray2);
  src_gray1 = getChannel(src, argv[3]);
  src_gray2 = getChannel(src2, argv[3]);

  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  imshow( source_window, src );

  opticalFlow( 0, 0 );

  waitKey(0);
  return(0);
}

Mat drawVectors(vector<Point2f> corners, vector<Point2f> nextPts, vector<uchar> status, vector<float> err){
  Mat image = src2.clone();
  for( int i=0; i<corners.size(); i++){
    if(status[i]==0 || err[i]>550){
      printf("Error is %f\n", err[i]);
      continue;
    }
    if (cornerMotion(corners[i], nextPts[i])){
      CvPoint p0 = cvPoint(
        cvRound( corners[i].x ),
        cvRound( corners[i].y )
      );
      CvPoint p1 = cvPoint(
        cvRound( nextPts[i].x),
        cvRound( nextPts[i].y)
      );
      arrowedLine( image, p0, p1, Scalar(100, 200, 150), 2, 8, 0, 0.2);
    }
  }
  return image;
}

/**
 * @function opticalFlow.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void opticalFlow( int, void* ){
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;
  vector<Point2f> nextPts;
  vector<uchar> status;
  vector<float> err;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = 0;
  double k = 0.04;

    /// Copy the source image
  Mat copy, copy2;
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
  Size subPixWinSize(10,10), winSize(31,31);
  TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
 
  cornerSubPix(
    src_gray1,
    corners,
    subPixWinSize,
    Size(-1, 1),
    termcrit
  );

  ///Call the Lucas Kanade algorith
  calcOpticalFlowPyrLK(
      src_gray1,
      src_gray2,
      corners,
      nextPts,
      status,
      err,
      winSize,
      3,
      termcrit,
      0,
      0.001
  );

  //Now make some image of what we are looking at:
  writeCorners(corners, nextPts);
  copy2 = drawVectors(corners, nextPts, status, err);

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 2;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(255, 170, 155), -1, 8, 0 ); }

  namedWindow(source_window, WINDOW_AUTOSIZE);
  namedWindow(source_windowB, WINDOW_AUTOSIZE);
  namedWindow(source_windowR, WINDOW_AUTOSIZE);
  imshow(source_window, copy);
  imshow(source_windowB, src_gray2);
  imshow(source_windowR, copy2);
}
