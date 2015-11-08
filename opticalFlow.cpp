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
vector<Vector> good_vectors;  //Result vectors of OpticalFlow method

int maxCorners = 350;
int maxTrackbar = 500;

const char* source_window = "ImageA";
const char* source_windowB = "ImageB";
const char* source_windowR = "Results";

/// Function header
void opticalFlow( int, void* );

/**
 * @function main
 */
int main( int, char** argv ){
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );

  if (strcmp(argv[3], "VB") == 0)
  {
    char arg[3] = "B";
    Mat result_image = src2.clone();
    vector<Vector> result_vectors;
    vector<Vector> other_vectors;
    vector<Vector> outliers_vectors;

    src_gray1 = getChannel(src, arg);
    src_gray2 = getChannel(src2, arg);
    opticalFlow(0, 0);
    result_vectors = good_vectors;
    
    strcpy(arg,"V");
    src_gray1 = getChannel(src, arg);
    src_gray2 = getChannel(src2, arg);
    opticalFlow(0, 0);
    other_vectors = good_vectors;

    //result_vectors.insert( result_vectors.end(), other_vectors.begin(), other_vectors.end() );
    //good_vectors = getGoodVectors(result_vectors, src2.size());
    //outliers_vectors = getOutliersVectors(result_vectors, src2.size());
    //char filename[24] = "Good vectors";
    //writeCorners(good_vectors, filename);
    //strcpy(filename,"Outliers vectors");
    //writeCorners(outliers_vectors, filename);
    result_image = drawVectors(result_vectors, result_image, Scalar(255, 0, 0));
    result_image = drawVectors(other_vectors, result_image, Scalar(0, 255, 255));
    namedWindow(source_windowR, WINDOW_AUTOSIZE);
    imshow(source_windowR, result_image);
    waitKey(0);

  }else{
    src_gray1 = getChannel(src, argv[3]);
    src_gray2 = getChannel(src2, argv[3]);
  }
  
  opticalFlow( 0, 0 );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  waitKey(0);
  return(0);
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

  /// Vectors
  vector<Vector> filter_vectors;
  vector<Vector> outliers_vectors;

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
  filter_vectors = getFilterVectors(corners, nextPts, status, err);
  good_vectors = getGoodVectors(filter_vectors, src.size());
  outliers_vectors = getOutliersVectors(filter_vectors, src.size());

  char filename[24] = "Good vectors";
  writeCorners(good_vectors, filename);
  strcpy(filename,"Outliers vectors");
  writeCorners(outliers_vectors, filename);

  copy2 = drawVectors(good_vectors, src2, Scalar(0, 255, 255));
  copy2 = drawVectors(outliers_vectors, copy2, Scalar(0, 0, 255));

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
