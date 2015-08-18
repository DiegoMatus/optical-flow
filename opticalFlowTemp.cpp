/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src2, src_gray1, src_gray2;

int maxCorners = 50;
int maxTrackbar = 500;

typedef Vec<int, 3> Vec3b;
RNG rng(12345);
const char* source_window = "ImageA";
const char* source_windowB = "ImageB";
const char* source_windowR = "Results";

/// Function header
void opticalFlow( int, void* );
void averageRGB(Mat *, Mat *);
void canalB();
void canalG();
void canalR();

/**
 * @function main
 */
int main( int, char** argv ){
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );

  cvtColor( src, src_gray1, COLOR_BGR2GRAY );
  cvtColor( src2, src_gray2, COLOR_BGR2GRAY );

  averageRGB(&src, &src_gray1);
  //canalB();
  //canalG();
  //canalR();
  /*Function compute average.
    1.- Promedio de RGB y sobreescribir el punto en la matriz
    2, 3, 4.- Buscar R/G/B y sobreescribir en la matriz
    SaveImage Guardar las images, hace video
    Capturar otro video más rapido.
    */
  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  imshow( source_window, src );

  opticalFlow( 0, 0 );

  waitKey(0);
  return(0);
}

void averageRGB(Mat *imageA, Mat *imageB){
  int average;
  for (int i = 0; i < 576; i++){
      for (int j = 0; j < 720; j++){
        average = (int)((imageA.at<cv::Vec3b>(i,j)[0] +
                  imageA.at<cv::Vec3b>(i,j)[1] +
                  imageA.at<cv::Vec3b>(i,j)[2])/3.0); 
        imageB.at<int>(i,j) = average;
      }
  }
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
  copy2 = src2.clone();

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
  for( int i=0; i<corners.size(); i++){
    if(status[i]==0 || err[i]>550){
      printf("Error is %f\n", err[i]);
      continue;
    }
    printf("Got it\n");
    CvPoint p0 = cvPoint(
      cvRound( corners[i].x ),
      cvRound( corners[i].y )
    );
    CvPoint p1 = cvPoint(
      cvRound( nextPts[i].x),
      cvRound( nextPts[i].y)
    );
    line( copy2, p0, p1, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 2, 8, 0);
  }

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); }

  namedWindow(source_window, WINDOW_AUTOSIZE);
  namedWindow(source_windowB, WINDOW_AUTOSIZE);
  namedWindow(source_windowR, WINDOW_AUTOSIZE);
  imshow(source_window, copy);
  imshow(source_windowB, src2);
  imshow(source_windowR, copy2);
}
