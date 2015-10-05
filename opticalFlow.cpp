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
//void canalX(Size, char *, Mat *, Mat *, Mat *, Mat *);

/**
 * @function main
 */
int main( int, char** argv ){
  int op = atoi(argv[3]);
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );
  Size tamanio = src.size();

  src_gray1 = getChannel(src, argv[3]);
  src_gray2 = getChannel(src2, argv[3]);
//  cvtColor( src, src_gray1, COLOR_BGR2GRAY);
//  cvtColor( src2, src_gray2, COLOR_BGR2GRAY);
//  cvtColor( src, hsv1, COLOR_BGR2HSV);
//  cvtColor( src2, hsv2, COLOR_BGR2HSV);
//  src = hsv1.clone();
//  src2 = hsv2.clone();

//  canalX(tamanio, argv[2], &src, &src2, &src_gray1, &src_gray2);
  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  imshow( source_window, src );

  opticalFlow( 0, 0 );

  waitKey(0);
  return(0);
}

Mat drawVectors(vector<Vector> good_vectors){
  bool is_outlier;
  Mat image = src2.clone();
  Vector max_vector = getMaxVector(good_vectors);
  float WINDOW = 50, ERROR_ANGLE = 5, ERROR_MAGNITUDE = 8;

  //is_outlier = isOutlier(image.size(), good_vectors, getMaxVector(good_vectors), 25, 10);
  for( int i=0; i<good_vectors.size(); i++){
      CvPoint p0 = cvPoint(
        cvRound( good_vectors[i].p1.x ),
        cvRound( good_vectors[i].p1.y )
      );
      CvPoint p1 = cvPoint(
        cvRound( good_vectors[i].p2.x),
        cvRound( good_vectors[i].p2.y)
      );
      if (good_vectors[i].p1 == max_vector.p1){
        //printf("Vector mayor: [%f,%f] - [%f,%f] Magnitud: %f\n", max_vector.p1.x, max_vector.p1.y, max_vector.p2.x, max_vector.p2.y, euclideanDistance(max_vector));
        arrowedLine( image, p0, p1, Scalar(255, 0, 0), 2, 8, 0, 0.2);        
      }

        if(isOutlier(src.size(), good_vectors, good_vectors[i], WINDOW, ERROR_ANGLE, ERROR_MAGNITUDE)){
          arrowedLine( image, p0, p1, Scalar(0, 0, 255), 2, 8, 0, 0.2);
        }else{
          arrowedLine( image, p0, p1, Scalar(0, 255, 255), 2, 8, 0, 0.2);
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
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  vector<Vector> good_vectors = getGoodVectors(corners, nextPts, status, err);
  writeCorners(good_vectors);
  copy2 = drawVectors(good_vectors);

  /// Draw corners detected
  int r = 2;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(255, 170, 155), -1, 8, 0 ); }

  namedWindow(source_window, WINDOW_AUTOSIZE);
  namedWindow(source_windowB, WINDOW_AUTOSIZE);
  namedWindow(source_windowR, WINDOW_AUTOSIZE);
  imshow(source_window,copy);
  imshow(source_windowB, src_gray2);
  imshow(source_windowR, copy2);
}
